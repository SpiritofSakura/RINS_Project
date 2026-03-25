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
    
    # Nodes from task1 package
    face_localizator_node = Node(
        package='task1',
        executable='face_localizator',
        name='face_localizator',
    )
    
    waypoint_navigator_node = Node(
        package='task1',
        executable='waypoint_navigator',
        name='waypoint_navigator',
    )
    
    # Delay waypoint_navigator startup by 5 seconds
    delayed_waypoint_navigator = TimerAction(
        period=5.0,
        actions=[waypoint_navigator_node],
    )
    
    # Create launch description and add all nodes
    ld = LaunchDescription([
        detect_people_node,
        face_localizator_node,
        delayed_waypoint_navigator,
    ])
    
    return ld
