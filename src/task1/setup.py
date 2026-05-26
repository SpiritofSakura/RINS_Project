from setuptools import find_packages, setup
from glob import glob

package_name = 'task1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/personnel', glob('config/personnel/*.png')),
        ('share/' + package_name + '/config/models', glob('config/models/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spectre',
    maintainer_email='spectre@todo.todo',
    description='Waypoint navigation package',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'waypoint_navigator = task1.waypoint_navigator:main',
            'simple_waypoints_nav = task1.simple_waypoints_nav:main',
            'face_localizator = task1.face_localizator:main',
            'path_listener = task1.path_listener:main',
            'ring_localizator = task1.ring_localizator:main',
            'robot_state_overlay = task1.robot_state_overlay:main',
            'behavior_manager = task1.behavior_manager:main',
            'detect_rings_v2 = task1.detect_rings_v2:main',
            'face_recognizer = task1.face_recognizer:main',
            'cylinder_localizator = task1.cylinder_localizator:main',
            'cylinder_debug_view = task1.cylinder_debug_view:main',
            'color_mask_viewer = task1.color_mask_viewer:main',
            'line_localizator = task1.line_localizator:main',
            'blue_line_explorer = task1.blue_line_explorer:main',
            'workstation_recorder = task1.workstation_recorder:main',
            'station_inspector = task1.station_inspector:main',
            'parallel_align = task1.parallel_align:main',
            'tile_detect = task1.tile_detect:main',
            'arm_camera_viewer = task1.arm_camera_viewer:main',
            'oakd_camera_viewer = task1.oakd_camera_viewer:main',
            'tile_classifier = task1.tile_classifier:main',
            'report_manager = task1.report:main',
            'orchestrator = task1.orchestrator:main',
        ],
    },
)
