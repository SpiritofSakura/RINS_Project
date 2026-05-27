from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch face detection pipeline + behavior_manager for blue-line face test."""

    detect_people_node = Node(
        package='dis_tutorial3',
        executable='detect_people.py',
        name='detect_people',
        output='screen',
        parameters=[{'device': 'cpu'}],
    )

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

    behavior_manager_node = Node(
        package='task1',
        executable='behavior_manager',
        name='behavior_manager',
        output='screen',
    )

    ld = LaunchDescription([
        detect_people_node,
        face_localizator_node,
        face_recognizer_node,
        behavior_manager_node,
    ])

    return ld
