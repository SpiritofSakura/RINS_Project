from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    real_robot_arg = DeclareLaunchArgument(
        'real_robot',
        default_value='true',
        description='Set to true when running on the real robot (uses Gemini topics)',
    )

    real_robot = LaunchConfiguration('real_robot')
    detector_backend = LaunchConfiguration('detector_backend')
    face_model = LaunchConfiguration('face_model')
    device = LaunchConfiguration('device')

    detector_backend_arg = DeclareLaunchArgument(
        'detector_backend',
        default_value='yolo',
        description='Face detector backend: yolo for GPU, haar for CPU fallback',
    )

    face_model_arg = DeclareLaunchArgument(
        'face_model',
        default_value='/home/zeta/RINS_Project/models/yolov8n-face-lindevs.pt',
        description='Path to a face-specific YOLO model',
    )

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='0',
        description='Ultralytics device. Use 0 for CUDA GPU, cpu for CPU.',
    )

    detect_people_node = Node(
        package='dis_tutorial3',
        executable='detect_people.py',
        name='detect_people',
        output='screen',
        parameters=[{
            'real_robot': real_robot,
            'detector_backend': detector_backend,
            'face_model': face_model,
            'device': ParameterValue(device, value_type=str),
        }],
    )

    face_localizator_node = Node(
        package='task1',
        executable='face_localizator',
        name='face_localizator',
        output='screen',
    )

    return LaunchDescription([
        real_robot_arg,
        detector_backend_arg,
        face_model_arg,
        device_arg,
        detect_people_node,
        face_localizator_node,
    ])
