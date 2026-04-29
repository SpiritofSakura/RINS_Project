from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """IRL patrol launch: waypoint navigator, behavior manager, and face detection."""

    behavior_manager_node = Node(
        package='task1',
        executable='behavior_manager',
        name='behavior_manager',
        output='screen',
    )

    robot_state_overlay_node = Node(
        package='task1',
        executable='robot_state_overlay',
        name='robot_state_overlay',
        output='screen',
    )

    waypoint_navigator_node = Node(
        package='task1',
        executable='waypoint_navigator',
        name='waypoint_navigator',
        output='screen',
        parameters=[{'waypoints_file': 'waypoints.yaml'}],
    )

    faces_test = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('task1'), 'launch', 'faces_test.launch.py')
        ),
        launch_arguments={'real_robot': 'true'}.items(),
    )

    return LaunchDescription([
        behavior_manager_node,
        waypoint_navigator_node,
        robot_state_overlay_node,
        faces_test,
    ])
