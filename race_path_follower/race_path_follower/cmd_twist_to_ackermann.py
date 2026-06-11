import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive


class CmdTwistToAckermann(Node):
    def __init__(self):
        super().__init__('cmd_twist_to_ackermann_blue')

        self.wheelbase = 0.257

        self.pub = self.create_publisher(
            AckermannDrive,
            '/blue_car/cmd_ackermann',
            10
        )

        self.sub = self.create_subscription(
            Twist,
            '/cmd_twist',
            self.callback,
            10
        )

    def callback(self, msg):
        v = msg.linear.x
        wz = msg.angular.z

        ack = AckermannDrive()
        ack.speed = v

        if abs(v) > 1e-6:
            ack.steering_angle = math.atan(self.wheelbase * wz / v)
        else:
            ack.steering_angle = 0.0

        self.pub.publish(ack)


def main(args=None):
    rclpy.init(args=args)

    node = CmdTwistToAckermann()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
