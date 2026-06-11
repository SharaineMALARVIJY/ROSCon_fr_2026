import rclpy
import math

from ackermann_msgs.msg import AckermannDrive
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class BlueDriver:
    def init(self, webots_node, properties):
        self.__is_active = False
        self.__node = None
        self.__tf_broadcaster = None

        self.__robot = webots_node.robot
        self.__timestep_ms = int(self.__robot.getBasicTimeStep())

        self.__vitesse_m_s = 0.0
        self.__steering_angle_rad = 0.0
        self.__measured_speed_m_s = 0.0

        self.__wheelbase = 0.257

        self.__odom_x = 0.0
        self.__odom_y = 0.0
        self.__odom_yaw = 0.0

        # Le yaw de pose vient toujours de l'IMU.
        self.__imu_yaw_offset = None

        # Seule la source de yaw_rate est configurable.
        # Valeurs possibles :
        #   - gyro
        #   - ackermann
        #   - imu_derivative
        self.__previous_imu_yaw = None

        self.__last_odom_time = None

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

        self.__imu_orientation = self.__robot.getDevice('inertial_unit')
        self.__gyro = self.__robot.getDevice('gyro')
        self.__imu_orientation.enable(self.__timestep_ms)
        self.__gyro.enable(self.__timestep_ms)

        # ROS interface
        if not rclpy.ok():
            rclpy.init(args=None)

        self.__node = rclpy.create_node('blue_driver')
        self.__tf_broadcaster = TransformBroadcaster(self.__node)

        # Paramètres ROS
        #
        # Le yaw de /odom est toujours calculé avec l'IMU.
        #
        # yaw_rate_source :
        #   - "gyro"           : yaw_rate depuis Gyro.z
        #   - "ackermann"      : yaw_rate = v / L * tan(delta)
        #   - "imu_derivative" : yaw_rate = dérivée du yaw IMU
        self.__node.declare_parameter('yaw_rate_source', 'ackermann')

        self.__yaw_rate_source = (
            self.__node.get_parameter('yaw_rate_source')
            .get_parameter_value()
            .string_value
        )


        if self.__yaw_rate_source not in ('gyro', 'ackermann', 'imu_derivative'):
            self.__node.get_logger().warn(
                f"[blue_driver] yaw_rate_source='{self.__yaw_rate_source}' invalide, "
                "fallback vers 'gyro'"
            )
            self.__yaw_rate_source = 'gyro'

        self.__node.get_logger().info(
            f"[blue_driver] yaw=imu, "
            f"yaw_rate_source={self.__yaw_rate_source}, "
        )

        self.__node.create_subscription(
            AckermannDrive,
            '/blue_car/cmd_ackermann',
            self.__cmd_ackermann_callback,
            1,
        )

        self.__joint_state_publisher = self.__node.create_publisher(
            JointState,
            '/blue_car/joint_states',
            10,
        )

        self.__current_speed_publisher = self.__node.create_publisher(
            Float32,
            '/blue_car/current_speed',
            10,
        )

        self.__odom_publisher = self.__node.create_publisher(
            Odometry,
            '/odom',
            10,
        )

        now = self.__node.get_clock().now()
        self.__last_odom_time = now

        self.__robot.setCruisingSpeed(0.0)
        self.__robot.setSteeringAngle(0.0)

        self.__is_active = True
        self.__node.get_logger().info('[blue_driver] Noeud créé')

    def __cmd_ackermann_callback(self, message):
        self.__vitesse_m_s = message.speed
        self.__steering_angle_rad = message.steering_angle

    def __normalize_angle(self, angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def __get_imu_yaw(self):
        roll, pitch, yaw = self.__imu_orientation.getRollPitchYaw()

        if self.__imu_yaw_offset is None:
            self.__imu_yaw_offset = yaw

        yaw = (yaw - self.__imu_yaw_offset)
        return self.__normalize_angle(yaw)

    def __get_gyro_yaw_rate(self):
        gx, gy, gz = self.__gyro.getValues()
        return gz

    def __get_ackermann_yaw_rate(self, v):
        return v / self.__wheelbase * math.tan(self.__steering_angle_rad)

    def __get_imu_derivative_yaw_rate(self, imu_yaw, dt):
        if self.__previous_imu_yaw is None:
            self.__previous_imu_yaw = imu_yaw
            return 0.0

        dyaw = self.__normalize_angle(imu_yaw - self.__previous_imu_yaw)
        self.__previous_imu_yaw = imu_yaw

        return dyaw / dt

    def __get_yaw_rate(self, imu_yaw, v, dt):
        if self.__yaw_rate_source == 'ackermann':
            return self.__get_ackermann_yaw_rate(v)

        if self.__yaw_rate_source == 'imu_derivative':
            return self.__get_imu_derivative_yaw_rate(imu_yaw, dt)

        return self.__get_gyro_yaw_rate()

    def __publish_joint_states(self):
        message = JointState()
        message.header.stamp = self.__node.get_clock().now().to_msg()
        message.name = self.__joint_state_names
        message.position = [
            -self.__front_left_steering_sensor.getValue(),
            -self.__front_right_steering_sensor.getValue(),
            self.__front_left_wheel_sensor.getValue(),
            self.__front_right_wheel_sensor.getValue(),
            self.__rear_left_wheel_sensor.getValue(),
            self.__rear_right_wheel_sensor.getValue(),
        ]
        self.__joint_state_publisher.publish(message)

    def __publish_current_speed(self):
        # Pour l'instant on garde la vitesse commandée, car getCurrentSpeed() renvoie 0
        # et getPosition()/getVelocity() nécessitent un Supervisor.
        self.__measured_speed_m_s = self.__vitesse_m_s

        message = Float32()
        message.data = self.__measured_speed_m_s
        self.__current_speed_publisher.publish(message)

    def __publish_odom(self):
        now = self.__node.get_clock().now()
        dt = (now - self.__last_odom_time).nanoseconds * 1e-9
        self.__last_odom_time = now

        if dt <= 0.0:
            return

        v = self.__measured_speed_m_s

        # Pose : le yaw vient toujours de l'IMU.
        self.__odom_yaw = self.__get_imu_yaw()

        # Twist : seul le yaw_rate est configurable.
        yaw_rate = self.__get_yaw_rate(self.__odom_yaw, v, dt)

        self.__odom_x += v * math.cos(self.__odom_yaw) * dt
        self.__odom_y += v * math.sin(self.__odom_yaw) * dt

        qz = math.sin(self.__odom_yaw * 0.5)
        qw = math.cos(self.__odom_yaw * 0.5)

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'

        msg.pose.pose.position.x = self.__odom_x
        msg.pose.pose.position.y = self.__odom_y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = yaw_rate

        self.__odom_publisher.publish(msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'

        tf_msg.transform.translation.x = self.__odom_x
        tf_msg.transform.translation.y = self.__odom_y
        tf_msg.transform.translation.z = 0.0

        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.__tf_broadcaster.sendTransform(tf_msg)

    def _shutdown_ros(self):
        if not getattr(self, '_BlueDriver__is_active', False):
            return

        self.__is_active = False

        try:
            self.__robot.setCruisingSpeed(0.0)
            self.__robot.setSteeringAngle(0.0)
        except Exception:
            pass

        if getattr(self, '_BlueDriver__node', None) is not None:
            self.__node.destroy_node()
            self.__node = None

        if rclpy.ok():
            rclpy.shutdown()

    def step(self):
        if not getattr(self, '_BlueDriver__is_active', False):
            return

        if not rclpy.ok():
            self._shutdown_ros()
            return

        try:
            rclpy.spin_once(self.__node, timeout_sec=0)
        except (KeyboardInterrupt, ExternalShutdownException):
            self._shutdown_ros()
            return

        self.__robot.setCruisingSpeed(self.__vitesse_m_s * 3.6)
        self.__robot.setSteeringAngle(-self.__steering_angle_rad)

        self.__publish_joint_states()
        self.__publish_current_speed()
        self.__publish_odom()

    def __del__(self):
        self._shutdown_ros()
