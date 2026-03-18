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
            'face_localizator = task1.face_localizator:main',
        ],
    },
)