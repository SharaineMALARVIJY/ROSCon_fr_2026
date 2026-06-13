from glob import glob
from setuptools import find_packages, setup

package_name = 'car_rosconfr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'click'],
    zip_safe=True,
    maintainer='NitROS',
    maintainer_email='sharaine.ma@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'car_drive = ' + package_name + '.car_drive_node:main',
            'car_teleop = ' + package_name + '.car_teleop_node:main',
            'teleop_ackermann = ' + package_name + '.teleop_ackermann:main',
            'ps4_teleop_ackermann = ' + package_name + '.ps4_teleop_ackermann:main',
            'follow_gap_blue = ' + package_name + '.follow_gap_blue:main',
            'dial_nav_blue = ' + package_name + '.dial_nav_blue:main',
            'front_scan_filter = ' + package_name + '.front_scan_filter:main',
            'simple_reactive_with_recovery_node = ' + package_name + '.simple_reactive_with_recovery_node:main',
        ],
    },
)
