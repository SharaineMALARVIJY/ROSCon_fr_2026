#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_c1_launch.py'
            )
        )
    )

    covapsy_node = Node(
        package='monPaquetCoVAPSYR',
        executable='CoVAPSy_cmdR',
        name='CoVAPSy_cmdR',
        output='screen',
        emulate_tty=True,
#        required=True
    )

    return LaunchDescription([
        rplidar_launch,
        covapsy_node
    ])
