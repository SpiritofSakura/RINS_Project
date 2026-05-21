from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value='irl-waypoints.yaml',
        description='Waypoints YAML file (relative to task1/config/)',
    )

    waypoints_file = LaunchConfiguration('waypoints_file')

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
        parameters=[{'waypoints_file': waypoints_file}],
    )

    # Auto-start patrol after 3 s so nav2 has time to come up
    start_patrol = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--times', '3',
                     '/patrol_command', 'std_msgs/msg/Bool', 'data: true'],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        waypoints_file_arg,
        behavior_manager_node,
        waypoint_navigator_node,
        start_patrol,
    ])
