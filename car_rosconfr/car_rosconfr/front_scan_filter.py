import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class FrontScanFilter(Node):
    def __init__(self):
        super().__init__('front_scan_filter')

        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.sub = self.create_subscription(
            LaserScan,
            '/yellow_car/scan',
            self.callback,
            10
        )

    def callback(self, msg):
        half_fov = math.radians(100.0)

        ranges = []
        intensities = []

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            angle = (angle + math.pi) % (2.0 * math.pi) - math.pi

            if -half_fov <= angle <= half_fov:
                ranges.append(r)

                if len(msg.intensities) > 0:
                    intensities.append(msg.intensities[i])

        if len(ranges) == 0:
            return

        out = LaserScan()
        out.header = msg.header

        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        if msg.angle_increment < 0.0:
            out.angle_min = half_fov
        else:
            out.angle_min = -half_fov

        out.angle_max = out.angle_min + (len(ranges) - 1) * out.angle_increment

        out.ranges = ranges

        if len(intensities) > 0:
            out.intensities = intensities

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)

    node = FrontScanFilter()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
