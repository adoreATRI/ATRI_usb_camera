import os

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    pkg_camera = get_package_share_directory("usb_camera")

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")

    params_path = os.path.join(pkg_camera, "config", "usb_camera_config.yaml")
    camera_params = ParameterFile(params_path, allow_substs=True)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )
    declare_start_rviz_cmd = DeclareLaunchArgument(
        "start_rviz",
        default_value="false",
        description="Start RViz with the camera display config",
    )
    declare_camera_path_cmd = DeclareLaunchArgument(
        "camera_path",
        default_value="",
        description="V4L2 camera device path",
    )

    usb_camera_node = Node(
        package="usb_camera",
        executable="usb_camera_node_exe",
        name="usb_camera_node",
        output="screen",
        parameters=[camera_params, {"use_sim_time": use_sim_time}],
    )

    rviz_node = Node(
        condition=IfCondition(start_rviz),
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_camera, "rviz", "usb_camera.rviz")],
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_start_rviz_cmd)
    ld.add_action(declare_camera_path_cmd)

    ld.add_action(usb_camera_node)
    ld.add_action(rviz_node)

    return ld
