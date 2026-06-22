#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class Odometry_node(Node):
    def __init__(self):
        super().__init__('odometry_node')

        self.declare_parameter('yaw_rate_source', 'gyro')
        self.declare_parameter('publish_tf', False)

        self.declare_parameter('speed_topic', '/yellow_car/current_speed')
        self.declare_parameter('imu_topic', '/yellow_car/imu')
        self.declare_parameter('cmd_topic', '/yellow_car/cmd_ackermann')
        self.declare_parameter('odom_topic', '/wheel/odom')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.yaw_rate_source = self.get_parameter('yaw_rate_source').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.wheelbase = 0.257
        self.v = 0.0
        self.steering = 0.0

        self.imu_yaw = 0.0
        self.previous_imu_yaw = None
        self.imu_yaw_rate = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = self.get_clock().now()

        self.create_subscription(
            Float32,
            self.get_parameter('speed_topic').value,
            self.speed_callback,
            10
        )

        self.create_subscription(
            Imu,
            self.get_parameter('imu_topic').value,
            self.imu_callback,
            10
        )

        self.create_subscription(
            AckermannDrive,
            self.get_parameter('cmd_topic').value,
            self.cmd_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            self.get_parameter('odom_topic').value,
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.02, self.update)  # 50 Hz

    def speed_callback(self, msg):
        self.v = msg.data

    def cmd_callback(self, msg):
        self.steering = msg.steering_angle

    def imu_callback(self, msg):
        self.imu_yaw_rate = msg.angular_velocity.z
        self.imu_yaw = self.quaternion_to_yaw(msg.orientation)

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, a):
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def get_yaw_rate(self, dt):
        if self.yaw_rate_source == 'ackermann':
            return self.v / self.wheelbase * math.tan(self.steering)

        if self.yaw_rate_source == 'imu_derivative':
            if self.previous_imu_yaw is None:
                self.previous_imu_yaw = self.imu_yaw
                return 0.0

            dyaw = self.normalize_angle(self.imu_yaw - self.previous_imu_yaw)
            self.previous_imu_yaw = self.imu_yaw
            return dyaw / dt

        return self.imu_yaw_rate

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0:
            return

        yaw_rate = self.get_yaw_rate(dt)

        # Ici on utilise le yaw IMU pour l'orientation
        self.yaw = self.imu_yaw

        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = yaw_rate

        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = now.to_msg()
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = Odometry_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()