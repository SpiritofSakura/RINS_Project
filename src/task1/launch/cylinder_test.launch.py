from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Minimal launch for testing cylinder detection: segmentation + localizator + debug view."""

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

    ld = LaunchDescription([
        arm_mover_node,
        cylinder_segmentation_node,
        cylinder_localizator_node,
        cylinder_debug_view_node,
    ])

    return ld
