from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    workstation_arg = DeclareLaunchArgument(
        "workstation",
        default_value="green",
        description="Workstation to inspect: green or red",
    )

    station_inspector_node = Node(
        package="task1",
        executable="station_inspector",
        name="station_inspector",
        output="screen",
        parameters=[{"workstation": LaunchConfiguration("workstation")}],
    )

    arm_camera_viewer_node = Node(
        package="task1",
        executable="arm_camera_viewer",
        name="arm_camera_viewer",
        output="screen",
    )

    line_localizator_node = Node(
        package="task1",
        executable="line_localizator",
        name="line_localizator",
        output="screen",
    )

    overlay_node = Node(
        package="task1",
        executable="robot_state_overlay",
        name="robot_state_overlay",
        output="screen",
    )

    oakd_viewer_node = Node(
        package="task1",
        executable="oakd_camera_viewer",
        name="oakd_camera_viewer",
        output="screen",
    )

    ld = LaunchDescription([
        workstation_arg,
        line_localizator_node,
        station_inspector_node,
        arm_camera_viewer_node,
        oakd_viewer_node,
        overlay_node,
    ])

    return ld
