#include "usb_camera/usb_camera_node.hpp"

// C++
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>

// V4L2
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <unistd.h>

// OpenCV
#include <opencv2/imgcodecs.hpp>

namespace usb_camera
{

USBCameraNode::USBCameraNode(const rclcpp::NodeOptions & options) : Node("usb_camera_node", options)
{
  RCLCPP_INFO(this->get_logger(), "USBCameraNode started.");

  camera_device_url_ = this->declare_parameter("camera_device_v4l_url", "");
  camera_info_url_ = this->declare_parameter("camera_info_url", "");
  image_topic_ = this->declare_parameter("image_topic", "usb_camera/image/compressed");
  image_raw_topic_ = this->declare_parameter("image_raw_topic", "usb_camera/image_raw");
  camera_info_topic_ = this->declare_parameter("camera_info_topic", "usb_camera/camera_info");
  frame_id_ = this->declare_parameter("frame_id", "camera_optical_frame");

  camera_info_manager_ =
    std::make_unique<camera_info_manager::CameraInfoManager>(this, "usb_camera");

  if (camera_info_manager_->validateURL(camera_info_url_)) {
    if (camera_info_manager_->loadCameraInfo(camera_info_url_)) {
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
      RCLCPP_INFO(this->get_logger(), "Loaded camera info from: %s", camera_info_url_.c_str());
    } else {
      RCLCPP_WARN(
        this->get_logger(), "Failed to load camera info from: %s", camera_info_url_.c_str());
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url_.c_str());
  }

  image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
    image_topic_, rclcpp::SensorDataQoS());
  image_raw_pub_ =
    this->create_publisher<sensor_msgs::msg::Image>(image_raw_topic_, rclcpp::SensorDataQoS());
  camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, rclcpp::SensorDataQoS());

  declareParameters();

  params_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&USBCameraNode::parametersCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Starting capture thread...");
  running_ = true;
  capture_thread_ = std::thread(&USBCameraNode::captureLoop, this);
}

USBCameraNode::~USBCameraNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down USBCameraNode...");

  running_ = false;

  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }

  closeCameraV4L2();

  RCLCPP_INFO(this->get_logger(), "USBCameraNode shut down complete.");
}

bool USBCameraNode::openCameraV4L2()
{
  std::lock_guard<std::mutex> lock(mutex_);

  v4l2_fd_ = open(camera_device_url_.c_str(), O_RDWR | O_NONBLOCK);
  if (v4l2_fd_ < 0) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to open camera %s: %s", camera_device_url_.c_str(),
      std::strerror(errno));
    return false;
  }

  struct v4l2_format fmt = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = img_width_;
  fmt.fmt.pix.height = img_height_;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;

  if (ioctl(v4l2_fd_, VIDIOC_S_FMT, &fmt) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set video format: %s", std::strerror(errno));
    closeCameraV4L2Locked();
    return false;
  }

  struct v4l2_streamparm parm = {};
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  parm.parm.capture.timeperframe.numerator = 1;
  parm.parm.capture.timeperframe.denominator = frame_rate_.load();
  if (ioctl(v4l2_fd_, VIDIOC_S_PARM, &parm) < 0) {
    RCLCPP_WARN(this->get_logger(), "Failed to set frame rate to %d", frame_rate_.load());
  }

  applyV4L2Controls();

  struct v4l2_requestbuffers req = {};
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (ioctl(v4l2_fd_, VIDIOC_REQBUFS, &req) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to request buffers: %s", std::strerror(errno));
    closeCameraV4L2Locked();
    return false;
  }

  if (req.count == 0) {
    RCLCPP_ERROR(this->get_logger(), "Camera driver did not allocate any buffers");
    closeCameraV4L2Locked();
    return false;
  }

  v4l2_buffers_.resize(req.count);
  for (unsigned int i = 0; i < req.count; ++i) {
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (ioctl(v4l2_fd_, VIDIOC_QUERYBUF, &buf) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to query buffer %d: %s", i, std::strerror(errno));
      closeCameraV4L2Locked();
      return false;
    }

    v4l2_buffers_[i].length = buf.length;
    v4l2_buffers_[i].start =
      mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, v4l2_fd_, buf.m.offset);

    if (v4l2_buffers_[i].start == MAP_FAILED) {
      RCLCPP_ERROR(this->get_logger(), "Failed to mmap buffer %d: %s", i, std::strerror(errno));
      closeCameraV4L2Locked();
      return false;
    }
  }

  for (unsigned int i = 0; i < req.count; ++i) {
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (ioctl(v4l2_fd_, VIDIOC_QBUF, &buf) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to queue buffer %d: %s", i, std::strerror(errno));
      closeCameraV4L2Locked();
      return false;
    }
  }

  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(v4l2_fd_, VIDIOC_STREAMON, &type) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to start stream: %s", std::strerror(errno));
    closeCameraV4L2Locked();
    return false;
  }

  return true;
}

void USBCameraNode::closeCameraV4L2()
{
  std::lock_guard<std::mutex> lock(mutex_);
  closeCameraV4L2Locked();
}

void USBCameraNode::closeCameraV4L2Locked()
{
  if (v4l2_fd_ >= 0) {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(v4l2_fd_, VIDIOC_STREAMOFF, &type);

    for (auto & buf : v4l2_buffers_) {
      if (buf.start != MAP_FAILED && buf.start != nullptr) {
        munmap(buf.start, buf.length);
      }
    }
    v4l2_buffers_.clear();

    close(v4l2_fd_);
    v4l2_fd_ = -1;
  }
}

bool USBCameraNode::setV4L2Control(int id, int value)
{
  struct v4l2_control ctrl = {};
  ctrl.id = id;
  ctrl.value = value;
  return ioctl(v4l2_fd_, VIDIOC_S_CTRL, &ctrl) >= 0;
}

void USBCameraNode::setV4L2ControlOrWarn(int id, int value, const char * name)
{
  if (!setV4L2Control(id, value)) {
    RCLCPP_WARN(
      this->get_logger(), "Failed to set %s to %d: %s", name, value, std::strerror(errno));
  }
}

void USBCameraNode::applyV4L2Controls()
{
  if (exposure_time_mode_.load() == 0) {
    setV4L2ControlOrWarn(V4L2_CID_EXPOSURE_AUTO, 1, "exposure_auto");
    setV4L2ControlOrWarn(V4L2_CID_EXPOSURE_ABSOLUTE, exposure_time_.load(), "exposure_absolute");
  } else {
    setV4L2ControlOrWarn(V4L2_CID_EXPOSURE_AUTO, 3, "exposure_auto");
  }

  setV4L2ControlOrWarn(V4L2_CID_BRIGHTNESS, brightness_.load(), "brightness");
  setV4L2ControlOrWarn(V4L2_CID_CONTRAST, contrast_.load(), "contrast");
  setV4L2ControlOrWarn(V4L2_CID_SATURATION, saturation_.load(), "saturation");
}

void USBCameraNode::declareParameters()
{
  img_width_ = this->declare_parameter("image_width", 1280);
  img_height_ = this->declare_parameter("image_height", 720);

  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;

  param_desc.description = "Frame rate (FPS)";
  param_desc.integer_range[0].from_value = 1;
  param_desc.integer_range[0].to_value = 60;
  frame_rate_ = this->declare_parameter("frame_rate", 60, param_desc);

  param_desc.description = "Exposure mode";
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 3;
  exposure_time_mode_ = this->declare_parameter("exposure_time_mode", 0, param_desc);

  param_desc.description = "Exposure time";
  param_desc.integer_range[0].from_value = 1;
  param_desc.integer_range[0].to_value = 10000;
  exposure_time_ = this->declare_parameter("exposure_time", 50, param_desc);

  param_desc.description = "Brightness";
  param_desc.integer_range[0].from_value = -64;
  param_desc.integer_range[0].to_value = 64;
  brightness_ = this->declare_parameter("brightness", 60, param_desc);

  param_desc.description = "Contrast";
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 100;
  contrast_ = this->declare_parameter("contrast", 50, param_desc);

  param_desc.description = "Saturation";
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 100;
  saturation_ = this->declare_parameter("saturation", 60, param_desc);
}

rcl_interfaces::msg::SetParametersResult USBCameraNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  std::lock_guard<std::mutex> lock(mutex_);
  bool camera_opened = (v4l2_fd_ >= 0);

  for (const auto & param : parameters) {
    const auto & name = param.get_name();

    if (name == "exposure_time_mode") {
      exposure_time_mode_ = param.as_int();
      if (camera_opened) {
        if (exposure_time_mode_ == 0) {
          setV4L2ControlOrWarn(V4L2_CID_EXPOSURE_AUTO, 1, "exposure_auto");
          setV4L2ControlOrWarn(
            V4L2_CID_EXPOSURE_ABSOLUTE, exposure_time_.load(), "exposure_absolute");
        } else {
          setV4L2ControlOrWarn(V4L2_CID_EXPOSURE_AUTO, 3, "exposure_auto");
        }
      }
      RCLCPP_INFO(this->get_logger(), "Exposure mode: %d", exposure_time_mode_.load());

    } else if (name == "exposure_time") {
      exposure_time_ = param.as_int();
      if (camera_opened && exposure_time_mode_ == 0) {
        setV4L2ControlOrWarn(
          V4L2_CID_EXPOSURE_ABSOLUTE, exposure_time_.load(), "exposure_absolute");
      }
      RCLCPP_INFO(this->get_logger(), "Exposure time: %d", exposure_time_.load());

    } else if (name == "brightness") {
      brightness_ = param.as_int();
      if (camera_opened) {
        setV4L2ControlOrWarn(V4L2_CID_BRIGHTNESS, brightness_.load(), "brightness");
      }
      RCLCPP_INFO(this->get_logger(), "Brightness: %d", brightness_.load());

    } else if (name == "contrast") {
      contrast_ = param.as_int();
      if (camera_opened) {
        setV4L2ControlOrWarn(V4L2_CID_CONTRAST, contrast_.load(), "contrast");
      }
      RCLCPP_INFO(this->get_logger(), "Contrast: %d", contrast_.load());

    } else if (name == "saturation") {
      saturation_ = param.as_int();
      if (camera_opened) {
        setV4L2ControlOrWarn(V4L2_CID_SATURATION, saturation_.load(), "saturation");
      }
      RCLCPP_INFO(this->get_logger(), "Saturation: %d", saturation_.load());
    }
  }

  return result;
}

void USBCameraNode::captureLoop()
{
  int fail_count = 0;
  constexpr int max_fail_count = 5;
  auto requeue_buffer = [this](v4l2_buffer & buffer) {
    if (ioctl(v4l2_fd_, VIDIOC_QBUF, &buffer) < 0) {
      RCLCPP_WARN(this->get_logger(), "Failed to requeue buffer: %s", std::strerror(errno));
      return false;
    }
    return true;
  };

  while (running_ && rclcpp::ok()) {
    if (!camera_connected_) {
      if (openCameraV4L2()) {
        camera_connected_ = true;
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;
      }
    }

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(v4l2_fd_, &fds);

    struct timeval tv = {};
    tv.tv_sec = 1;
    tv.tv_usec = 0;

    int r = select(v4l2_fd_ + 1, &fds, nullptr, nullptr, &tv);
    if (r < 0 && errno == EINTR) {
      continue;
    }
    if (r <= 0) {
      if (r < 0) {
        RCLCPP_WARN(this->get_logger(), "Camera select failed: %s", std::strerror(errno));
      }
      fail_count++;
      if (fail_count >= max_fail_count) {
        RCLCPP_ERROR(this->get_logger(), "Camera timeout, reconnecting...");
        closeCameraV4L2();
        camera_connected_ = false;
        fail_count = 0;
      }
      continue;
    }

    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(v4l2_fd_, VIDIOC_DQBUF, &buf) < 0) {
      if (errno == EAGAIN) {
        continue;
      }
      RCLCPP_WARN(this->get_logger(), "Failed to dequeue buffer: %s", std::strerror(errno));
      fail_count++;
      if (fail_count >= max_fail_count) {
        closeCameraV4L2();
        camera_connected_ = false;
        fail_count = 0;
      }
      continue;
    }

    fail_count = 0;

    if (buf.index >= v4l2_buffers_.size()) {
      RCLCPP_WARN(this->get_logger(), "Camera returned invalid buffer index: %u", buf.index);
      requeue_buffer(buf);
      continue;
    }

    const auto & v4l2_buffer = v4l2_buffers_[buf.index];
    if (buf.bytesused == 0 || buf.bytesused > v4l2_buffer.length) {
      RCLCPP_WARN(
        this->get_logger(), "Camera returned invalid frame size: %u bytes", buf.bytesused);
      requeue_buffer(buf);
      continue;
    }

    sensor_msgs::msg::CompressedImage image_msg;
    image_msg.header.stamp = this->now();
    image_msg.header.frame_id = frame_id_;
    image_msg.format = "jpeg";

    const uint8_t * jpeg_data = static_cast<const uint8_t *>(v4l2_buffer.start);
    image_msg.data.assign(jpeg_data, jpeg_data + buf.bytesused);
    camera_info_msg_.header = image_msg.header;

    image_pub_->publish(image_msg);

    if (image_raw_pub_->get_subscription_count() > 0) {
      cv::Mat decoded_image = cv::imdecode(image_msg.data, cv::IMREAD_COLOR);
      if (decoded_image.empty()) {
        RCLCPP_WARN(this->get_logger(), "Failed to decode MJPEG frame");
      } else {
        if (!decoded_image.isContinuous()) {
          decoded_image = decoded_image.clone();
        }

        sensor_msgs::msg::Image image_raw_msg;
        image_raw_msg.header = image_msg.header;
        image_raw_msg.height = static_cast<uint32_t>(decoded_image.rows);
        image_raw_msg.width = static_cast<uint32_t>(decoded_image.cols);
        image_raw_msg.encoding = "bgr8";
        image_raw_msg.is_bigendian = false;
        image_raw_msg.step = static_cast<uint32_t>(decoded_image.step);
        image_raw_msg.data.assign(
          decoded_image.data, decoded_image.data + image_raw_msg.step * image_raw_msg.height);
        image_raw_pub_->publish(image_raw_msg);
      }
    }

    camera_info_pub_->publish(camera_info_msg_);

    requeue_buffer(buf);
  }

  closeCameraV4L2();
}
}  // namespace usb_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(usb_camera::USBCameraNode)
