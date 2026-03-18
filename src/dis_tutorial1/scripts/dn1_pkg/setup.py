from setuptools import setup
from glob import glob
import os

package_name = 'dn1_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='triki',
    maintainer_email='tristan.flander30@gmail.com',
    description='Homework1 package with custom messages and services',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = dn1_pkg.publisher_node:main',
            'subscriber_node = dn1_pkg.subscriber_node:main',
            'service_node = dn1_pkg.service_node:main',
            'client_node = dn1_pkg.client_node:main',
        ],
    },
)