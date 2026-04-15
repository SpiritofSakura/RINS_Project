from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Minimal IRL patrol launch: waypoint navigator + behavior manager only."""

    behavior_manager_node = Node(
        package='task1',
        executable='behavior_manager',
        name='behavior_manager',
        output='screen',
    )

    waypoint_navigator_node = Node(
        package='task1',
        executable='waypoint_navigator',
        name='waypoint_navigator',
        output='screen',
        parameters=[{'waypoints_file': 'irl-waypoints.yaml'}],
    )

    return LaunchDescription([
        behavior_manager_node,
        waypoint_navigator_node,
    ])
