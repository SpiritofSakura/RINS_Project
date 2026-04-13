from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the task1 application with all required nodes."""

    real_robot_arg = DeclareLaunchArgument(
        'real_robot', default_value='false', choices=['true', 'false'],
        description='Use real robot configuration (IRL waypoints, depth camera, etc.)'
    )
    real_robot = LaunchConfiguration('real_robot')

    waypoints_file = PythonExpression([
        "'irl-waypoints.yaml' if '", real_robot, "' == 'true' else 'waypoints.yaml'"
    ])

    # Node from dis_tutorial3 package
    detect_people_node = Node(
        package='dis_tutorial3',
        executable='detect_people.py',
        name='detect_people',
        parameters=[
            {'enabled': True, 'real_robot': real_robot},
        ],
    )
    
    detect_rings_node = Node(
        package='task1',
        executable='detect_rings_v2',
        name='detect_rings_v2',
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
        parameters=[{'real_robot': real_robot}],
    )


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
        parameters=[{'waypoints_file': waypoints_file}],
    )

    robot_state_overlay_node = Node(
        package='task1',
        executable='robot_state_overlay',
        name='robot_state_overlay',
        output='screen',
    )
    
    # Delay ring detector and localizator startup by 6 seconds
    delayed_detect_rings = TimerAction(
        period=6.0,
        actions=[detect_rings_node],
    )

    delayed_ring_localizator = TimerAction(
        period=6.0,
        actions=[ring_localizator_node],
    )
    
    # Create launch description and add all nodes
    ld = LaunchDescription([
        real_robot_arg,
        detect_people_node,
        face_localizator_node,
        robot_state_overlay_node,
        behavior_manager_node,
        waypoint_navigator_node,
        delayed_detect_rings,
        delayed_ring_localizator,
    ])
    
    return ld
