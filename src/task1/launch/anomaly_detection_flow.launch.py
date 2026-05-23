from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    color_mask_viewer_node = Node(
        package='task1',
        executable='color_mask_viewer',
        name='color_mask_viewer',
        output='screen',
    )

    line_localizator_node = Node(
        package='task1',
        executable='line_localizator',
        name='line_localizator',
        output='screen',
    )

    workstation_recorder_node = Node(
        package='task1',
        executable='workstation_recorder',
        name='workstation_recorder',
        output='screen',
    )

    ld = LaunchDescription([
        color_mask_viewer_node,
        line_localizator_node,
        workstation_recorder_node,
    ])

    return ld
