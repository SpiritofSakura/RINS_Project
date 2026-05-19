from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch the task1 application with all required nodes."""

    real_robot_arg = DeclareLaunchArgument(
        'real_robot', default_value='false', choices=['true', 'false'],
        description='Use real robot configuration (IRL waypoints, depth camera, etc.)'
    )
    detector_backend_arg = DeclareLaunchArgument(
        'detector_backend', default_value='yolo',
        description='Face detector backend: yolo for GPU, haar for CPU fallback'
    )
    face_model_arg = DeclareLaunchArgument(
        'face_model', default_value='/home/zeta/RINS_Project/models/yolov8n-face-lindevs.pt',
        description='Path to a face-specific YOLO model'
    )
    device_arg = DeclareLaunchArgument(
        'device', default_value='0',
        description='Ultralytics device. Use 0 for CUDA GPU, cpu for CPU.'
    )
    ring_model_arg = DeclareLaunchArgument(
        'ring_model', default_value='/home/zeta/ring_yolo/ring_det/weights/best.pt',
        description='Path to trained YOLO ring model'
    )
    ring_conf_arg = DeclareLaunchArgument(
        'ring_conf', default_value='0.30',
        description='YOLO confidence threshold for ring detections'
    )
    ring_imgsz_arg = DeclareLaunchArgument(
        'ring_imgsz', default_value='640',
        description='YOLO inference image size for ring detections'
    )
    ring_hz_arg = DeclareLaunchArgument(
        'ring_hz', default_value='12.0',
        description='Maximum ring detector inference rate'
    )
    require_depth_hole_arg = DeclareLaunchArgument(
        'require_depth_hole', default_value='true',
        description='Reject YOLO ring boxes that do not look hollow in depth'
    )
    enable_colour_fallback_arg = DeclareLaunchArgument(
        'enable_colour_fallback', default_value='true',
        description='Use HSV ellipse fallback when YOLO misses obvious coloured rings'
    )
    yolo_confirm_frames_arg = DeclareLaunchArgument(
        'yolo_confirm_frames', default_value='2',
        description='Consecutive frames required before accepting a YOLO ring'
    )
    colour_confirm_frames_arg = DeclareLaunchArgument(
        'colour_confirm_frames', default_value='4',
        description='Consecutive frames required before accepting a colour fallback ring'
    )
    ring_image_topic_arg = DeclareLaunchArgument(
        'ring_image_topic', default_value='',
        description='Override ring detector RGB image topic'
    )
    ring_depth_topic_arg = DeclareLaunchArgument(
        'ring_depth_topic', default_value='',
        description='Override ring detector depth image topic'
    )
    ring_camera_info_topic_arg = DeclareLaunchArgument(
        'ring_camera_info_topic', default_value='',
        description='Override ring detector camera info topic'
    )
    real_robot = LaunchConfiguration('real_robot')
    detector_backend = LaunchConfiguration('detector_backend')
    face_model = LaunchConfiguration('face_model')
    device = LaunchConfiguration('device')
    ring_model = LaunchConfiguration('ring_model')
    ring_conf = LaunchConfiguration('ring_conf')
    ring_imgsz = LaunchConfiguration('ring_imgsz')
    ring_hz = LaunchConfiguration('ring_hz')
    require_depth_hole = LaunchConfiguration('require_depth_hole')
    enable_colour_fallback = LaunchConfiguration('enable_colour_fallback')
    yolo_confirm_frames = LaunchConfiguration('yolo_confirm_frames')
    colour_confirm_frames = LaunchConfiguration('colour_confirm_frames')
    ring_image_topic = LaunchConfiguration('ring_image_topic')
    ring_depth_topic = LaunchConfiguration('ring_depth_topic')
    ring_camera_info_topic = LaunchConfiguration('ring_camera_info_topic')

    waypoints_file = PythonExpression([
        "'irl-waypoints.yaml' if '", real_robot, "' == 'true' else 'waypoints.yaml'"
    ])

    # Node from dis_tutorial3 package
    detect_people_node = Node(
        package='dis_tutorial3',
        executable='detect_people.py',
        name='detect_people',
        parameters=[
            {
                'enabled': True,
                'real_robot': real_robot,
                'detector_backend': detector_backend,
                'face_model': face_model,
                'device': ParameterValue(device, value_type=str),
            },
        ],
    )
    
    detect_rings_node = Node(
        package='task1',
        executable='detect_rings_yolo',
        name='detect_rings_yolo',
        output='screen',
        parameters=[{
            'real_robot': real_robot,
            'model_path': ring_model,
            'device': ParameterValue(device, value_type=str),
            'conf': ParameterValue(ring_conf, value_type=float),
            'imgsz': ParameterValue(ring_imgsz, value_type=int),
            'inference_hz': ParameterValue(ring_hz, value_type=float),
            'require_depth_hole': ParameterValue(require_depth_hole, value_type=bool),
            'enable_colour_fallback': ParameterValue(enable_colour_fallback, value_type=bool),
            'yolo_confirm_frames': ParameterValue(yolo_confirm_frames, value_type=int),
            'colour_confirm_frames': ParameterValue(colour_confirm_frames, value_type=int),
            'image_topic': ParameterValue(ring_image_topic, value_type=str),
            'depth_topic': ParameterValue(ring_depth_topic, value_type=str),
            'camera_info_topic': ParameterValue(ring_camera_info_topic, value_type=str),
        }],
    )


    # Nodes from task1 package
    face_localizator_node = Node(
        package='task1',
        executable='face_localizator',
        name='face_localizator',
    )


    ring_localizator_node = Node(
        package='task1',
        executable='ring_localizator',
        name='ring_localizator',
        output='screen',
        parameters=[{'real_robot': real_robot}],
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
        parameters=[{'waypoints_file': waypoints_file}],
    )

    robot_state_overlay_node = Node(
        package='task1',
        executable='robot_state_overlay',
        name='robot_state_overlay',
        output='screen',
    )
    
    # Delay ring detector and localizator startup by 6 seconds
    delayed_detect_rings = TimerAction(
        period=6.0,
        actions=[detect_rings_node],
    )

    delayed_ring_localizator = TimerAction(
        period=6.0,
        actions=[ring_localizator_node],
    )
    
    # Create launch description and add all nodes
    ld = LaunchDescription([
        real_robot_arg,
        detector_backend_arg,
        face_model_arg,
        device_arg,
        ring_model_arg,
        ring_conf_arg,
        ring_imgsz_arg,
        ring_hz_arg,
        require_depth_hole_arg,
        enable_colour_fallback_arg,
        yolo_confirm_frames_arg,
        colour_confirm_frames_arg,
        ring_image_topic_arg,
        ring_depth_topic_arg,
        ring_camera_info_topic_arg,
        detect_people_node,
        face_localizator_node,
        robot_state_overlay_node,
        behavior_manager_node,
        waypoint_navigator_node,
        delayed_detect_rings,
        delayed_ring_localizator,
    ])
    
    return ld
