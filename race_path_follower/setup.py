import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'race_path_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        ('share/' + package_name + '/behavior_tree', glob('behavior_tree/*')),
        ('share/' + package_name + '/params', glob('params/*')),
        ('share/' + package_name + '/maps', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NitROS',
    maintainer_email='sharaine.ma@gmail.com',
    description='Manage how the car make laps in the race with nav2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'race_node=race_path_follower.race_node:main',
            'improved_race_node=race_path_follower.improved_race_node:main',
            'race_node_sliding_path=race_path_follower.race_node_sliding_path:main',
            'cmd_twist_to_ackermann = race_path_follower.cmd_twist_to_ackermann:main',
        ],
    },
)
