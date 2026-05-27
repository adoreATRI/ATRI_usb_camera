#ifndef USB_CAMERA__USB_CAMERA_NODE_HPP_
#define USB_CAMERA__USB_CAMERA_NODE_HPP_

// C++
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ROS2
#include "camera_info_manager/camera_info_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace usb_camera
{

class USBCameraNode : public rclcpp::Node
{
public:
  explicit USBCameraNode(const rclcpp::NodeOptions & options);
  ~USBCameraNode();

private:
  struct V4L2Buffer
  {
    void * start{nullptr};
    size_t length{0};
  };

  void declareParameters();
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  void captureLoop();
  bool openCameraV4L2();
  void closeCameraV4L2();
  void closeCameraV4L2Locked();

  void applyV4L2Controls();
  bool setV4L2Control(int id, int value);
  void setV4L2ControlOrWarn(int id, int value, const char * name);

  std::mutex mutex_;
  int v4l2_fd_{-1};
  std::vector<V4L2Buffer> v4l2_buffers_;

  // 节点参数
  // 设备和内参路径
  std::string camera_device_url_;
  std::string camera_info_url_;
  // 话题参数
  std::string image_topic_;
  std::string image_raw_topic_;
  std::string camera_info_topic_;
  std::string frame_id_;
  // 采集线程参数
  int max_fail_count_;
  int v4l2_buffers_requested_count_;
  double restart_time_;
  double frame_timeout_time_;

  // 参数回调
  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
  int img_width_{1280};
  int img_height_{720};
  std::atomic<int> frame_rate_{60};
  std::atomic<int> exposure_time_{50};
  std::atomic<int> exposure_time_mode_{0};
  std::atomic<int> brightness_{60};
  std::atomic<int> contrast_{50};
  std::atomic<int> saturation_{60};

  std::atomic<bool> running_{false};
  std::atomic<bool> camera_connected_{false};
  std::thread capture_thread_;

  sensor_msgs::msg::CameraInfo camera_info_msg_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
};

}  // namespace usb_camera

#endif  // USB_CAMERA__USB_CAMERA_NODE_HPP_
