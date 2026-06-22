from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar_topic = LaunchConfiguration('lidar_topic')
    ackermann_topic = LaunchConfiguration('ackermann_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_topic',
            default_value='/yellow_car/scan',
            description='Topic de souscription du lidar.',
        ),
        DeclareLaunchArgument(
            'ackermann_topic',
            default_value='/yellow_car/cmd_ackermann',
            description='Topic de publication des commandes Ackermann.',
        ),
        Node(
            package='car_rosconfr',
            executable='car_drive',
            name='car_drive',
            output='screen',
            parameters=[{
                'lidar_topic': lidar_topic,
                'ackermann_topic': ackermann_topic,
            }],
        ),
    ])
