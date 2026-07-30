from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'field_control'

setup(
    name=package_name,
    version='0.0.0',
    package_dir={'': 'src'},
    packages=find_packages(where='src'),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dhcn-1',
    maintainer_email='qbao1607@gmail.com',
    description='Generic ROS 2 package for real-time per-link 3D solid obstacle distance calculation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'obstacle_distance_node = obstacle_distance.obstacle_distance_node:main',
            'field_control_node = obstacle_distance.obstacle_distance_node:main'
        ],
    },
)
