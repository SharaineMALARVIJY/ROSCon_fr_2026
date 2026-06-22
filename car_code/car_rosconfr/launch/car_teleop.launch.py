from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic_name = LaunchConfiguration('topic_name')
    linear_speed_increment = LaunchConfiguration('linear_speed_increment')
    steering_angle_increment = LaunchConfiguration('steering_angle_increment')

    return LaunchDescription([
        DeclareLaunchArgument(
            'topic_name',
            default_value='/yellow_car/cmd_ackermann',
            description='Topic de publication des commandes Ackermann.',
        ),
        DeclareLaunchArgument(
            'linear_speed_increment',
            default_value='0.5',
            description='Incrément de vitesse linéaire en m/s.',
        ),
        DeclareLaunchArgument(
            'steering_angle_increment',
            default_value='0.25',
            description='Incrément de braquage en rad.',
        ),
        Node(
            package='car_rosconfr',
            executable='car_teleop',
            name='car_teleop',
            output='screen',
            parameters=[{
                'topic_name': topic_name,
                'linear_speed_increment': linear_speed_increment,
                'steering_angle_increment': steering_angle_increment,
            }],
        ),
    ])
