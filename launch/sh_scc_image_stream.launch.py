from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import sh_common_constants

def generate_launch_description():
    desc = LaunchDescription()
    desc.add_action(DeclareLaunchArgument(
        "video_device_id",
        default_value="0"
    ))
    desc.add_action(Node(
        package="image_publisher",
        executable="image_publisher_node",
        name="sh_scc_image_stream",
        namespace="/smart_home",
        remappings=[
            ("image_raw", sh_common_constants.topics.SCC_CAMERA_IMAGE),
            ("camera_info", "_scc_unused_camera_info"),
        ],
        arguments=[LaunchConfiguration("video_device_id")],
        on_exit=Shutdown(),
    ))
    desc.add_action(LogInfo(msg=LaunchConfiguration("video_device_id")))
    return desc
