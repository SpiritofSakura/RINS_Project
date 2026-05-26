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
            {'device': 'cuda'},
        ],
    )
    
    detect_rings_node = Node(
        package='task1',
        executable='detect_rings_v2',
        name='detect_rings_v2',
        output='screen',
    )

    cylinder_segmentation_node = Node(
        package='dis_tutorial5',
        executable='cylinder_segmentation',
        name='cylinder_segmentation',
        output='screen',
    )

    cylinder_localizator_node = Node(
        package='task1',
        executable='cylinder_localizator',
        name='cylinder_localizator',
        output='screen',
    )


    # Nodes from task1 package
    face_localizator_node = Node(
        package='task1',
        executable='face_localizator',
        name='face_localizator',
        output='screen',
    )

    face_recognizer_node = Node(
        package='task1',
        executable='face_recognizer',
        name='face_recognizer',
        output='screen',
    )


    ring_localizator_node = Node(
        package='task1',
        executable='ring_localizator',
        name='ring_localizator',
        output='screen',
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
    )

    blue_line_explorer_node = Node(
        package='task1',
        executable='blue_line_explorer',
        name='blue_line_explorer',
        output='screen',
    )

    robot_state_overlay_node = Node(
        package='task1',
        executable='robot_state_overlay',
        name='robot_state_overlay',
        output='screen',
    )
    
    # Delay detectors and localizators by 6 seconds (wait for TF + sensors)
    delayed_detect_rings = TimerAction(
        period=6.0,
        actions=[detect_rings_node],
    )

    delayed_ring_localizator = TimerAction(
        period=6.0,
        actions=[ring_localizator_node],
    )

    delayed_cylinder_segmentation = TimerAction(
        period=6.0,
        actions=[cylinder_segmentation_node],
    )

    delayed_cylinder_localizator = TimerAction(
        period=6.0,
        actions=[cylinder_localizator_node],
    )

    orchestrator_node = Node(
        package='task1',
        executable='orchestrator',
        name='orchestrator',
        output='screen',
    )

    # Create launch description and add all nodes
    ld = LaunchDescription([
        detect_people_node,
        face_localizator_node,
        face_recognizer_node,
        robot_state_overlay_node,
        behavior_manager_node,
        waypoint_navigator_node,
        blue_line_explorer_node,
        delayed_detect_rings,
        delayed_ring_localizator,
        delayed_cylinder_segmentation,
        delayed_cylinder_localizator,
        orchestrator_node,
    ])
    
    return ld
