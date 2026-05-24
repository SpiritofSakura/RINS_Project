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

    arm_mover_node = Node(
        package="dis_tutorial7",
        executable="arm_mover_actions.py",
        name="arm_mover",
        output="screen",
    )

    arm_camera_viewer_node = Node(
        package="task1",
        executable="arm_camera_viewer",
        name="arm_camera_viewer",
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
        station_inspector_node,
        arm_mover_node,
        arm_camera_viewer_node,
        oakd_viewer_node,
    ])

    return ld
