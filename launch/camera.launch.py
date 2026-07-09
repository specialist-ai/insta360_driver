from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("name", default_value="insta360_camera",
                              description="camera identity: topic prefix /<name>/image_raw and frame_id"),
        DeclareLaunchArgument("serial", default_value="",
                              description="device serial to open; empty = first found"),
        DeclareLaunchArgument("mode", default_value="720",
                              description="720 | 1280 | 1920 (fisheye size)"),
        DeclareLaunchArgument("pixel_format", default_value="bgr8",
                              description="bgr8 or mono8"),
        Node(
            package="insta360_camera",
            executable="camera_node",
            name="insta360_camera",
            output="screen",
            parameters=[{
                "cameras": [[LaunchConfiguration("name"), ":", LaunchConfiguration("serial")]],
                # ParameterValue(value_type=str): launch would otherwise
                # coerce "720" to int and the string-typed parameter throws
                "mode": ParameterValue(LaunchConfiguration("mode"),
                                       value_type=str),
                "pixel_format": ParameterValue(
                    LaunchConfiguration("pixel_format"), value_type=str),
            }],
        ),
    ])
