from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    real_robot_arg = DeclareLaunchArgument(
        'real_robot',
        default_value='false',
        description='Set to true when running on the real robot (uses Gemini topics)',
    )

    real_robot = LaunchConfiguration('real_robot')

    detect_people_node = Node(
        package='dis_tutorial3',
        executable='detect_people.py',
        name='detect_people',
        output='screen',
        parameters=[{'real_robot': real_robot}],
    )

    face_localizator_node = Node(
        package='task1',
        executable='face_localizator',
        name='face_localizator',
        output='screen',
    )

    return LaunchDescription([
        real_robot_arg,
        detect_people_node,
        face_localizator_node,
    ])
