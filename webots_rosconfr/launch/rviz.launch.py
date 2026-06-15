import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('webots_rosconfr')

    rviz_config = os.path.join(
        package_dir,
        'config',
        'RPP.rviz',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        parameters=[
            {'use_sim_time': True},
        ],
        output='screen',
    )

    return LaunchDescription([
        rviz,
    ])