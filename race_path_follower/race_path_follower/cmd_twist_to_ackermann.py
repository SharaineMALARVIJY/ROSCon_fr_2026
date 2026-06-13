#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive


class CmdTwistToAckermannMux(Node):
    """
    Convertisseur + mux de commandes pour yellow_car.

    Entrées:
      - /cmd_twist : commande normale Nav2 en geometry_msgs/Twist
      - /yellow_car/cmd_ackermann_recovery : commande prioritaire recovery en AckermannDrive

    Sortie:
      - /yellow_car/cmd_ackermann : commande finale pour la voiture

    Principe:
      - si une commande recovery récente existe, elle est prioritaire ;
      - sinon on convertit /cmd_twist en AckermannDrive.
    """

    def __init__(self):
        super().__init__('cmd_twist_to_ackermann')

        self.declare_parameter('cmd_twist_topic', '/cmd_twist')
        self.declare_parameter('recovery_topic', '/yellow_car/cmd_ackermann_recovery')
        self.declare_parameter('output_topic', '/yellow_car/cmd_ackermann')
        self.declare_parameter('recovery_timeout', 0.30)

        self.cmd_twist_topic = str(self.get_parameter('cmd_twist_topic').value)
        self.recovery_topic = str(self.get_parameter('recovery_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.recovery_timeout = float(self.get_parameter('recovery_timeout').value)

        self.wheelbase = 0.257
        self.last_recovery_msg = None
        self.last_recovery_time = None

        self.pub = self.create_publisher(AckermannDrive, self.output_topic, 10)

        self.twist_sub = self.create_subscription(
            Twist,
            self.cmd_twist_topic,
            self.twist_callback,
            10,
        )

        self.recovery_sub = self.create_subscription(
            AckermannDrive,
            self.recovery_topic,
            self.recovery_callback,
            10,
        )

        # self.get_logger().info(
        #     'CmdTwistToAckermannMux démarré: '
        #     f'nav2={self.cmd_twist_topic}, '
        #     f'recovery={self.recovery_topic}, '
        #     f'output={self.output_topic}, '
        #     f'timeout={self.recovery_timeout:.2f}s'
        # )

    def recovery_is_active(self):
        if self.last_recovery_time is None:
            return False

        age = (self.get_clock().now() - self.last_recovery_time).nanoseconds * 1e-9
        return age <= self.recovery_timeout

    def twist_to_ackermann(self, msg):
        v = float(msg.linear.x)
        wz = float(msg.angular.z)

        ack = AckermannDrive()
        ack.speed = v

        if abs(v) > 1e-6:
            ack.steering_angle = math.atan(self.wheelbase * wz / v)
        else:
            ack.steering_angle = 0.0

        return ack

    def recovery_callback(self, msg):
        self.last_recovery_msg = msg
        self.last_recovery_time = self.get_clock().now()
        self.pub.publish(msg)

    def twist_callback(self, msg):
        # Tant qu'une commande recovery récente arrive, on ignore Nav2.
        if self.recovery_is_active() and self.last_recovery_msg is not None:
            self.pub.publish(self.last_recovery_msg)
            self.get_logger().info(
                'RECOVERY prioritaire: commande Nav2 ignorée',
                throttle_duration_sec=0.5,
            )
            return

        ack = self.twist_to_ackermann(msg)
        self.pub.publish(ack)


def main(args=None):
    rclpy.init(args=args)

    node = CmdTwistToAckermannMux()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = AckermannDrive()
        stop.speed = 0.0
        stop.steering_angle = 0.0
        node.pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
