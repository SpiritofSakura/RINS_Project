from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    """Launch for testing cylinder detection + spill check."""

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

    behavior_manager_node = Node(
        package='task1',
        executable='behavior_manager',
        name='behavior_manager',
        output='screen',
    )

    pointcloud_viewer_node = Node(
        package='task1',
        executable='pointcloud_viewer',
        name='pointcloud_viewer',
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

    ld = LaunchDescription([
        behavior_manager_node,
        pointcloud_viewer_node,
        delayed_cylinder_segmentation,
        delayed_cylinder_localizator,
    ])

    return ld
