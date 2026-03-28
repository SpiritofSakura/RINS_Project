from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the task1 application with all required nodes."""
    
    # Node from dis_tutorial3 package
    detect_people_node = Node(
        package='dis_tutorial3',
        executable='detect_people.py',
        name='detect_people',
        parameters=[
            {'enabled': True},
        ],
    )
    
    detect_rings_node = Node(
        package='dis_tutorial5',
        executable='detect_rings.py',
        name='detect_rings',
        output='screen',
    )


    # Nodes from task1 package
    face_localizator_node = Node(
        package='task1',
        executable='face_localizator',
        name='face_localizator',
    )


    ring_localizator_node = Node(
        package='task1',
        executable='ring_localizator',
        name='ring_localizator',
        output='screen',
    )

    waypoint_navigator_node = Node(
        package='task1',
        executable='waypoint_navigator',
        name='waypoint_navigator',
    )

    robot_state_overlay_node = Node(
        package='task1',
        executable='robot_state_overlay',
        name='robot_state_overlay',
        output='screen',
    )
    
    # Delay waypoint_navigator startup by 5 seconds
    delayed_waypoint_navigator = TimerAction(
        period=5.0,
        actions=[waypoint_navigator_node],
    )

    # Delay ring_localizator startup by 2 seconds
    delayed_ring_localizator = TimerAction(
        period=2.0,
        actions=[ring_localizator_node],
    )
    
    # Create launch description and add all nodes
    ld = LaunchDescription([
        detect_people_node,
        detect_rings_node,
        face_localizator_node,
        robot_state_overlay_node,
        delayed_waypoint_navigator,
        delayed_ring_localizator,
    ])
    
    return ld
