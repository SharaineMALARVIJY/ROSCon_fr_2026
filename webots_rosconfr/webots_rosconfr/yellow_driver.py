import rclpy
import math
import random
from collections import deque

from ackermann_msgs.msg import AckermannDrive
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from ament_index_python.packages import get_package_share_directory


class YellowDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep_ms = int(self.__robot.getBasicTimeStep())
        self.__dt = self.__timestep_ms / 1000.0

        # Derniere commande recue via ROS
        self.__cmd_speed_m_s = 0.0
        self.__cmd_steering_angle_rad = 0.0

        # Commande réellement appliquée au robot après latence + dynamique
        self.__applied_speed_m_s = 0.0
        self.__applied_steering_angle_rad = 0.0

        self.__is_active = True
        self.__joint_state_names = [
            'front_left_steering_joint',
            'front_right_steering_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
        ]

        self.__front_left_steering_sensor = self.__robot.getDevice('left_steer_sensor')
        self.__front_right_steering_sensor = self.__robot.getDevice('right_steer_sensor')
        self.__front_left_wheel_sensor = self.__robot.getDevice('left_front_sensor')
        self.__front_right_wheel_sensor = self.__robot.getDevice('right_front_sensor')
        self.__rear_left_wheel_sensor = self.__robot.getDevice('left_rear_sensor')
        self.__rear_right_wheel_sensor = self.__robot.getDevice('right_rear_sensor')

        self.__front_left_steering_sensor.enable(self.__timestep_ms)
        self.__front_right_steering_sensor.enable(self.__timestep_ms)
        self.__front_left_wheel_sensor.enable(self.__timestep_ms)
        self.__front_right_wheel_sensor.enable(self.__timestep_ms)
        self.__rear_left_wheel_sensor.enable(self.__timestep_ms)
        self.__rear_right_wheel_sensor.enable(self.__timestep_ms)

        self.__accelerometer = self.__robot.getDevice('accelerometer')
        self.__imu_orientation = self.__robot.getDevice('inertial_unit')
        self.__gyro = self.__robot.getDevice('gyro')

        self.__accelerometer.enable(self.__timestep_ms)
        self.__imu_orientation.enable(self.__timestep_ms)
        self.__gyro.enable(self.__timestep_ms)

        # ROS interface
        package_dir = get_package_share_directory('webots_rosconfr')
        rclpy.init(args=[
            '--ros-args',
            '--params-file',
            package_dir+'/config/yellow_driver_params.yaml'
        ])

        self.__node = rclpy.create_node('yellow_driver')

        # -------------------------
        # Parametres de realisme
        # -------------------------
        # Tous les bruits sont des ecarts-types de bruit gaussien.
        self.__node.declare_parameter('cmd_latency_s', 0.0)              # retard sur cmd_ackermann
        self.__node.declare_parameter('max_accel_m_s2', 0.0)            # 0.0 = accel instantanee
        self.__node.declare_parameter('max_decel_m_s2', 0.0)            # 0.0 = freinage instantane
        self.__node.declare_parameter('max_steering_rate_rad_s', 0.0)   # 0.0 = direction instantanee

        self.__node.declare_parameter('steering_cmd_noise_std_rad', 0.0)     # bruit sur ordre de direction applique
        self.__node.declare_parameter('odom_speed_noise_std_m_s', 0.0)       # bruit sur /current_speed
        self.__node.declare_parameter('odom_wheel_pos_noise_std_rad', 0.0)   # bruit encodeurs roues dans /joint_states
        self.__node.declare_parameter('odom_steer_pos_noise_std_rad', 0.0)   # bruit capteurs direction dans /joint_states
        self.__node.declare_parameter('gyro_yaw_rate_noise_std_rad_s', 0.0)


        # Lecture initiale des parametres
        self.__reload_realism_params()
        self.__cmd_queue = deque()

        self.__node.create_subscription(
            AckermannDrive,
            '/yellow_car/cmd_ackermann',
            self.__cmd_ackermann_callback,
            1,
        )
        self.__joint_state_publisher = self.__node.create_publisher(
            JointState,
            '/yellow_car/joint_states',
            10,
        )
        self.__current_speed_publisher = self.__node.create_publisher(
            Float32,
            '/yellow_car/current_speed',
            10,
        )
        self.__imu_publisher = self.__node.create_publisher(
            Imu,
            '/yellow_car/imu',
            10,
        )
        self.__node.get_logger().info('Noeud créé')
        self.__robot.setCruisingSpeed(0.0)  # En km/h
        self.__robot.setSteeringAngle(0.0)  # En radian

    def __reload_realism_params(self):
        self.__cmd_latency_s = float(self.__node.get_parameter('cmd_latency_s').value)
        self.__max_accel_m_s2 = float(self.__node.get_parameter('max_accel_m_s2').value)
        self.__max_decel_m_s2 = float(self.__node.get_parameter('max_decel_m_s2').value)
        self.__max_steering_rate_rad_s = float(self.__node.get_parameter('max_steering_rate_rad_s').value)

        self.__steering_cmd_noise_std_rad = float(self.__node.get_parameter('steering_cmd_noise_std_rad').value)
        self.__odom_speed_noise_std_m_s = float(self.__node.get_parameter('odom_speed_noise_std_m_s').value)
        self.__odom_wheel_pos_noise_std_rad = float(self.__node.get_parameter('odom_wheel_pos_noise_std_rad').value)
        self.__odom_steer_pos_noise_std_rad = float(self.__node.get_parameter('odom_steer_pos_noise_std_rad').value)
        self.__gyro_yaw_rate_noise_std_rad_s = float(self.__node.get_parameter('gyro_yaw_rate_noise_std_rad_s').value)

    @staticmethod
    def __clamp(value, min_value, max_value):
        return max(min_value, min(value, max_value))

    @staticmethod
    def __noise(std):
        if std <= 0.0:
            return 0.0
        return random.gauss(0.0, std)

    def __cmd_ackermann_callback(self, message):
        now_s = self.__node.get_clock().now().nanoseconds * 1e-9
        self.__cmd_queue.append((now_s, float(message.speed), float(message.steering_angle)))

    def __get_delayed_command(self):
        """Retourne la commande retardee par cmd_latency_s."""
        now_s = self.__node.get_clock().now().nanoseconds * 1e-9
        target_time_s = now_s - self.__cmd_latency_s

        # On consomme les commandes plus vieilles que la latence demandee.
        while len(self.__cmd_queue) >= 2 and self.__cmd_queue[1][0] <= target_time_s:
            self.__cmd_queue.popleft()

        if self.__cmd_queue and self.__cmd_queue[0][0] <= target_time_s:
            _, self.__cmd_speed_m_s, self.__cmd_steering_angle_rad = self.__cmd_queue[0]

        return self.__cmd_speed_m_s, self.__cmd_steering_angle_rad

    def __update_applied_command(self):
        target_speed_m_s, target_steering_rad = self.__get_delayed_command()

        # Limitation acceleration / deceleration
        speed_error = target_speed_m_s - self.__applied_speed_m_s
        if speed_error >= 0.0 and self.__max_accel_m_s2 > 0.0:
            max_delta_v = self.__max_accel_m_s2 * self.__dt
            speed_delta = self.__clamp(speed_error, -max_delta_v, max_delta_v)
        elif speed_error < 0.0 and self.__max_decel_m_s2 > 0.0:
            max_delta_v = self.__max_decel_m_s2 * self.__dt
            speed_delta = self.__clamp(speed_error, -max_delta_v, max_delta_v)
        else:
            speed_delta = speed_error
        self.__applied_speed_m_s += speed_delta

        # Limitation vitesse de braquage du servo
        steering_error = target_steering_rad - self.__applied_steering_angle_rad
        if self.__max_steering_rate_rad_s > 0.0:
            max_delta_steer = self.__max_steering_rate_rad_s * self.__dt
            steering_delta = self.__clamp(steering_error, -max_delta_steer, max_delta_steer)
        else:
            steering_delta = steering_error
        self.__applied_steering_angle_rad += steering_delta

        # Bruit sur la direction réellement envoyée à Webots
        noisy_steering_rad = self.__applied_steering_angle_rad + self.__noise(self.__steering_cmd_noise_std_rad)
        return self.__applied_speed_m_s, noisy_steering_rad

    def __publish_joint_states(self):
        message = JointState()
        message.header.stamp = self.__node.get_clock().now().to_msg()
        message.name = self.__joint_state_names
        message.position = [
            -self.__front_left_steering_sensor.getValue() + self.__noise(self.__odom_steer_pos_noise_std_rad),
            -self.__front_right_steering_sensor.getValue() + self.__noise(self.__odom_steer_pos_noise_std_rad),
            self.__front_left_wheel_sensor.getValue() + self.__noise(self.__odom_wheel_pos_noise_std_rad),
            self.__front_right_wheel_sensor.getValue() + self.__noise(self.__odom_wheel_pos_noise_std_rad),
            self.__rear_left_wheel_sensor.getValue() + self.__noise(self.__odom_wheel_pos_noise_std_rad),
            self.__rear_right_wheel_sensor.getValue() + self.__noise(self.__odom_wheel_pos_noise_std_rad),
        ]
        self.__joint_state_publisher.publish(message)

    def __publish_current_speed(self):
        message = Float32()
        # getCurrentSpeed() ne marche pas correctement dans ta simu, donc on publie la vitesse appliquee.
        message.data = self.__applied_speed_m_s + self.__noise(self.__odom_speed_noise_std_m_s)
        self.__current_speed_publisher.publish(message)

    def __publish_imu(self):
        roll, pitch, yaw = self.__imu_orientation.getRollPitchYaw()
        gx, gy, gz = self.__gyro.getValues()
        ax, ay, az = self.__accelerometer.getValues()

        msg = Imu()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # RPY -> quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        msg.orientation.w = cr * cp * cy + sr * sp * sy
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz + self.__noise(self.__gyro_yaw_rate_noise_std_rad_s)

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        self.__imu_publisher.publish(msg)

    def _shutdown_ros(self):
        if not self.__is_active:
            return
        self.__is_active = False
        self.__robot.setCruisingSpeed(0.0)
        self.__robot.setSteeringAngle(0.0)
        if hasattr(self, '_YellowDriver__node') and self.__node is not None:
            self.__node.destroy_node()
            self.__node = None
        if rclpy.ok():
            rclpy.shutdown()

    def step(self):
        if not self.__is_active or not rclpy.ok():
            self._shutdown_ros()
            return
        try:
            rclpy.spin_once(self.__node, timeout_sec=0)
        except (KeyboardInterrupt, ExternalShutdownException):
            self._shutdown_ros()
            return

        # Permet de changer les parametres à chaud avec ros2 param set.
        self.__reload_realism_params()

        speed_m_s, steering_rad = self.__update_applied_command()
        self.__robot.setCruisingSpeed(speed_m_s * 3.6)  # Conversion m/s -> km/h
        self.__robot.setSteeringAngle(-steering_rad)

        self.__publish_joint_states()
        self.__publish_current_speed()
        self.__publish_imu()

    def __del__(self):
        self._shutdown_ros()
