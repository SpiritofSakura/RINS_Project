from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    """Minimal launch for testing cylinder detection: segmentation + localizator + debug view + behavior manager."""

    arm_mover_node = Node(
        package='dis_tutorial7',
        executable='arm_mover_actions.py',
        name='arm_mover',
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

    cylinder_debug_view_node = Node(
        package='task1',
        executable='cylinder_debug_view',
        name='cylinder_debug_view',
        output='screen',
        parameters=[{'show_windows': True}],
    )

    behavior_manager_node = Node(
        package='task1',
        executable='behavior_manager',
        name='behavior_manager',
        output='screen',
    )

    delayed_cylinder_segmentation = TimerAction(
        period=6.0,
        actions=[cylinder_segmentation_node],
    )

    delayed_cylinder_localizator = TimerAction(
        period=6.0,
        actions=[cylinder_localizator_node],
    )

    delayed_cylinder_debug_view = TimerAction(
        period=6.0,
        actions=[cylinder_debug_view_node],
    )

    ld = LaunchDescription([
        arm_mover_node,
        behavior_manager_node,
        delayed_cylinder_segmentation,
        delayed_cylinder_localizator,
        delayed_cylinder_debug_view,
    ])

    return ld
