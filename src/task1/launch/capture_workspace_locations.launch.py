from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="toYAML",
        description="workstation_recorder mode: normal or toYAML",
    )

    color_mask_viewer_node = Node(
        package="task1",
        executable="color_mask_viewer",
        name="color_mask_viewer",
        output="screen",
    )

    line_localizator_node = Node(
        package="task1",
        executable="line_localizator",
        name="line_localizator",
        output="screen",
    )

    workstation_recorder_node = Node(
        package="task1",
        executable="workstation_recorder",
        name="workstation_recorder",
        output="screen",
        arguments=["--mode", LaunchConfiguration("mode")],
    )

    arm_camera_viewer_node = Node(
        package="task1",
        executable="arm_camera_viewer",
        name="arm_camera_viewer",
        output="screen",
    )

    ld = LaunchDescription([
        mode_arg,
        color_mask_viewer_node,
        line_localizator_node,
        workstation_recorder_node,
        arm_camera_viewer_node,
    ])

    return ld
