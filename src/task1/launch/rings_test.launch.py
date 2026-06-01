from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    real_robot_arg = DeclareLaunchArgument(
        'real_robot',
        default_value='false',
        description='Set to true when running on the real robot (uses Gemini topics)',
    )
    ring_model_arg = DeclareLaunchArgument(
        'ring_model',
        default_value='/home/zeta/RINS_Project/models/best.pt',
        description='Path to trained YOLO ring model',
    )
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='0',
        description='Ultralytics device. Use 0 for CUDA GPU, cpu for CPU.',
    )
    ring_conf_arg = DeclareLaunchArgument(
        'ring_conf',
        default_value='0.30',
        description='YOLO confidence threshold for ring detections',
    )
    ring_imgsz_arg = DeclareLaunchArgument(
        'ring_imgsz',
        default_value='640',
        description='YOLO inference image size',
    )
    inference_hz_arg = DeclareLaunchArgument(
        'inference_hz',
        default_value='12.0',
        description='Maximum ring detector inference rate',
    )
    debug_image_arg = DeclareLaunchArgument(
        'debug_image',
        default_value='true',
        description='Publish /ring_debug_image',
    )
    display_windows_arg = DeclareLaunchArgument(
        'display_windows',
        default_value='true',
        description='Open OpenCV Ring YOLO and Disparity windows',
    )
    require_depth_hole_arg = DeclareLaunchArgument(
        'require_depth_hole',
        default_value='true',
        description='Reject YOLO boxes that do not look hollow in depth',
    )
    enable_colour_fallback_arg = DeclareLaunchArgument(
        'enable_colour_fallback',
        default_value='true',
        description='Use HSV ellipse fallback when YOLO misses obvious coloured rings',
    )
    yolo_confirm_frames_arg = DeclareLaunchArgument(
        'yolo_confirm_frames',
        default_value='2',
        description='Consecutive frames required before accepting a YOLO ring',
    )
    colour_confirm_frames_arg = DeclareLaunchArgument(
        'colour_confirm_frames',
        default_value='4',
        description='Consecutive frames required before accepting a colour fallback ring',
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='',
        description='Override ring detector RGB image topic',
    )
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='',
        description='Override ring detector depth image topic',
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='',
        description='Override ring detector camera info topic',
    )

    real_robot = LaunchConfiguration('real_robot')
    ring_model = LaunchConfiguration('ring_model')
    device = LaunchConfiguration('device')
    ring_conf = LaunchConfiguration('ring_conf')
    ring_imgsz = LaunchConfiguration('ring_imgsz')
    inference_hz = LaunchConfiguration('inference_hz')
    debug_image = LaunchConfiguration('debug_image')
    display_windows = LaunchConfiguration('display_windows')
    require_depth_hole = LaunchConfiguration('require_depth_hole')
    enable_colour_fallback = LaunchConfiguration('enable_colour_fallback')
    yolo_confirm_frames = LaunchConfiguration('yolo_confirm_frames')
    colour_confirm_frames = LaunchConfiguration('colour_confirm_frames')
    image_topic = LaunchConfiguration('image_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

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
            'inference_hz': ParameterValue(inference_hz, value_type=float),
            'debug_image': ParameterValue(debug_image, value_type=bool),
            'display_windows': ParameterValue(display_windows, value_type=bool),
            'require_depth_hole': ParameterValue(require_depth_hole, value_type=bool),
            'enable_colour_fallback': ParameterValue(enable_colour_fallback, value_type=bool),
            'yolo_confirm_frames': ParameterValue(yolo_confirm_frames, value_type=int),
            'colour_confirm_frames': ParameterValue(colour_confirm_frames, value_type=int),
            'image_topic': ParameterValue(image_topic, value_type=str),
            'depth_topic': ParameterValue(depth_topic, value_type=str),
            'camera_info_topic': ParameterValue(camera_info_topic, value_type=str),
        }],
    )

    ring_localizator_node = Node(
        package='task1',
        executable='ring_localizator',
        name='ring_localizator',
        output='screen',
        parameters=[{'real_robot': real_robot}],
    )

    simple_waypoints_nav_node = Node(
        package='task1',
        executable='simple_waypoints_nav',
        name='simple_waypoints_nav',
        output='screen',
    )

    return LaunchDescription([
        real_robot_arg,
        ring_model_arg,
        device_arg,
        ring_conf_arg,
        ring_imgsz_arg,
        inference_hz_arg,
        debug_image_arg,
        display_windows_arg,
        require_depth_hole_arg,
        enable_colour_fallback_arg,
        yolo_confirm_frames_arg,
        colour_confirm_frames_arg,
        image_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        detect_rings_node,
        ring_localizator_node,
        # simple_waypoints_nav_node,
    ])
