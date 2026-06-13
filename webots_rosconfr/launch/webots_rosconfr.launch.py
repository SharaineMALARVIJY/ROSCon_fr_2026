import os

import launch
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def make_webots_controller(robot_name, urdf_path, bridge_node_name):
    return WebotsController(
        robot_name=robot_name,
        parameters=[
            {
                'robot_description': urdf_path,
            },
        ],
        remappings=[
            ('__node', bridge_node_name),
        ],
        respawn=True,
    )


def make_robot_state_publisher(robot_name, urdf_path):
    with open(urdf_path, 'r') as file:
        robot_description = file.read()

    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'{robot_name}_state_publisher',
        namespace=robot_name,
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ],
        remappings=[
            ('joint_states', f'/{robot_name}/joint_states'),
        ],
        output='screen',
    )


def generate_launch_description():
    package_dir = get_package_share_directory('webots_rosconfr')

    world_path = os.path.join(
        package_dir,
        'worlds',
        'Piste_CoVAPSy_2025a.wbt'
    )

    yellow_urdf_path = os.path.join(
        package_dir,
        'resource',
        'TT02_jaune.urdf'
    )

    blue_urdf_path = os.path.join(
        package_dir,
        'resource',
        'TT02_bleu.urdf'
    )

    webots = WebotsLauncher(
        world=world_path,
        mode='realtime',
        gui=True,
    )

    yellow_webots_bridge = make_webots_controller(
        robot_name='yellow_car',
        urdf_path=yellow_urdf_path,
        bridge_node_name='yellow_car',
    )

    blue_webots_bridge = make_webots_controller(
        robot_name='blue_car',
        urdf_path=blue_urdf_path,
        bridge_node_name='blue_webots_bridge',
    )

    yellow_state_publisher = make_robot_state_publisher(
        robot_name='yellow_car',
        urdf_path=yellow_urdf_path,
    )

    blue_state_publisher = make_robot_state_publisher(
        robot_name='blue_car',
        urdf_path=blue_urdf_path,
    )

    car_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('car_odometry'),
                'launch',
                'car_odometry.launch.py'
            )
        )
    )

    front_scan_filter = Node(
        package='car_rosconfr',
        executable='front_scan_filter',
        name='front_scan_filter',
        output='screen',
    )

    shutdown_on_webots_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=webots,
            on_exit=[
                EmitEvent(event=launch.events.Shutdown())
            ],
        )
    )

    return LaunchDescription([
        webots,

        yellow_webots_bridge,
        # blue_webots_bridge,

        yellow_state_publisher,
        # blue_state_publisher,

        car_odometry_launch,
        front_scan_filter,

        shutdown_on_webots_exit,
    ])
