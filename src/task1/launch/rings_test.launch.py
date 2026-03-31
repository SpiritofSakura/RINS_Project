from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the rings test with ring_localizator, detect_rings_v2, and simple waypoint navigator."""
    
    # Ring detection node v2 (Hough circle-based) from task1 package
    detect_rings_node = Node(
        package='task1',
        executable='detect_rings_v2',
        name='detect_rings_v2',
        output='screen',
    )

    # Ring localization node from task1 package
    ring_localizator_node = Node(
        package='task1',
        executable='ring_localizator',
        name='ring_localizator',
        output='screen',
    )

    # Simple waypoint navigator node from task1 package
    simple_waypoints_nav_node = Node(
        package='task1',
        executable='simple_waypoints_nav',
        name='simple_waypoints_nav',
        output='screen',
    )

    # Create launch description and add all nodes
    ld = LaunchDescription([
        detect_rings_node,
        ring_localizator_node,
        # simple_waypoints_nav_node,
    ])
    
    return ld

