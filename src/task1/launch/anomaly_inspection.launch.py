from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    workstation_arg = DeclareLaunchArgument(
        "workstation",
        default_value="green",
        description="Workstation to inspect: green or red",
    )
    use_yaml_arg = DeclareLaunchArgument(
        "use_yaml",
        default_value="false",
        description="Use YAML waypoints instead of orchestrator",
    )
    use_orchestrator_arg = DeclareLaunchArgument(
        "use_orchestrator",
        default_value="true",
        description="Use orchestrator for waypoints",
    )

    station_inspector_node = Node(
        package="task1",
        executable="station_inspector",
        name="station_inspector",
        output="screen",
        parameters=[
            {"workstation": LaunchConfiguration("workstation")},
            {"use_yaml": LaunchConfiguration("use_yaml")},
            {"use_orchestrator": LaunchConfiguration("use_orchestrator")},
        ],
    )

    arm_camera_viewer_node = Node(
        package="task1",
        executable="arm_camera_viewer",
        name="arm_camera_viewer",
        output="screen",
    )

    line_localizator_node = Node(
        package="task1",
        executable="line_localizator",
        name="line_localizator",
        output="screen",
    )

    workstation_recorder_node = Node(
        package="task1",
        executable="workstation_recorder",
        name="workstation_recorder",
        output="screen",
    )

    orchestrator_node = Node(
        package="task1",
        executable="orchestrator",
        name="orchestrator",
        output="screen",
    )

    overlay_node = Node(
        package="task1",
        executable="robot_state_overlay",
        name="robot_state_overlay",
        output="screen",
    )

    oakd_viewer_node = Node(
        package="task1",
        executable="oakd_camera_viewer",
        name="oakd_camera_viewer",
        output="screen",
    )

    ld = LaunchDescription([
        workstation_arg,
        use_yaml_arg,
        use_orchestrator_arg,
        line_localizator_node,
        workstation_recorder_node,
        orchestrator_node,
        station_inspector_node,
        arm_camera_viewer_node,
        oakd_viewer_node,
        overlay_node,
    ])

    return ld
