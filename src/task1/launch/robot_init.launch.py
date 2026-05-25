from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    arm_camera_viewer_node = Node(
        package="task1",
        executable="arm_camera_viewer",
        name="arm_camera_viewer",
        output="screen",
    )

    behavior_manager_node = Node(
        package="task1",
        executable="behavior_manager",
        name="behavior_manager",
        output="screen",
    )

    oakd_viewer_node = Node(
        package="task1",
        executable="oakd_camera_viewer",
        name="oakd_camera_viewer",
        output="screen",
    )

    orchestrator_node = Node(
        package="task1",
        executable="orchestrator",
        name="orchestrator",
        output="screen",
    )

    report_node = Node(
        package="task1",
        executable="report_manager",
        name="report_manager",
        output="screen",
    )

    overlay_node = Node(
        package="task1",
        executable="robot_state_overlay",
        name="robot_state_overlay",
        output="screen",
    )

    waypoint_navigator_node = Node(
        package="task1",
        executable="waypoint_navigator",
        name="waypoint_navigator",
        output="screen",
    )

    blue_line_explorer_node = Node(
        package="task1",
        executable="blue_line_explorer",
        name="blue_line_explorer",
        output="screen",
    )

    ld = LaunchDescription([
        arm_camera_viewer_node,
        behavior_manager_node,
        oakd_viewer_node,
        orchestrator_node,
        report_node,
        overlay_node,
        waypoint_navigator_node,
        blue_line_explorer_node,
    ])

    return ld
