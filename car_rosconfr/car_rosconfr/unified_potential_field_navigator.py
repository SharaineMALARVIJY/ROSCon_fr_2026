#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDrive


class UnifiedPotentialFieldNavigator(Node):

    def __init__(self):
        super().__init__('unified_potential_field_navigator_blue')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_topic', '/yellow_car/cmd_ackermann')
        self.declare_parameter('emergency_topic', '/emergency_stop')

        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.emergency_topic = self.get_parameter('emergency_topic').value

        self.declare_parameter('max_speed', 1.6)
        self.declare_parameter('min_speed', 0.25)
        self.declare_parameter('max_steering_angle', 0.28)
        self.declare_parameter('invert_steering', True)

        self.declare_parameter('fov_deg', 200.0)
        self.declare_parameter('influence_distance', 4.0)
        self.declare_parameter('k_repulsive', 0.6)
        self.declare_parameter('k_attractive', 1.0)
        self.declare_parameter('smoothing_alpha', 0.3)
        self.declare_parameter('front_obstacle_threshold', 2.0)
        self.declare_parameter('critical_obstacle_distance', 0.5)
        self.declare_parameter('repulsive_deadzone', 0.1)

        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        self.invert_steering = bool(self.get_parameter('invert_steering').value)

        self.fov_deg = float(self.get_parameter('fov_deg').value)
        self.influence_distance = float(self.get_parameter('influence_distance').value)
        self.k_repulsive = float(self.get_parameter('k_repulsive').value)
        self.k_attractive = float(self.get_parameter('k_attractive').value)
        self.smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
        self.front_obstacle_threshold = float(self.get_parameter('front_obstacle_threshold').value)
        self.critical_obstacle_distance = float(self.get_parameter('critical_obstacle_distance').value)
        self.repulsive_deadzone = float(self.get_parameter('repulsive_deadzone').value)

        self.smoothed_speed = 0.0
        self.smoothed_steering = 0.0
        self.emergency_stop = False

        self.cmd_pub = self.create_publisher(AckermannDrive, self.cmd_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.emergency_sub = self.create_subscription(Bool, self.emergency_topic, self.emergency_callback, 10)

        self.get_logger().info('Unified Potential Field Navigator BLUE started')
        self.get_logger().info(f'scan_topic = {self.scan_topic}')
        self.get_logger().info(f'cmd_topic  = {self.cmd_topic}')
        self.get_logger().info(f'FOV frontal = {self.fov_deg} deg')

    def apply_steering_deadzone(self, repulsive_magnitude, steering_norm):
        if abs(repulsive_magnitude) <= self.repulsive_deadzone:
            scale = (abs(repulsive_magnitude) / self.repulsive_deadzone) ** 2
        else:
            scale = 1.0

        return steering_norm * scale

    def publish_ackermann(self, speed, steering_angle):
        msg = AckermannDrive()

        if self.invert_steering:
            steering_angle = -steering_angle

        msg.speed = float(speed)
        msg.steering_angle = float(steering_angle)

        self.cmd_pub.publish(msg)

    def stop_vehicle(self):
        self.publish_ackermann(0.0, 0.0)

    def scan_callback(self, msg: LaserScan):

        try:
            if self.emergency_stop:
                self.stop_vehicle()
                return

            ranges = np.array(msg.ranges, dtype=float)
            n = ranges.size

            if n == 0:
                self.stop_vehicle()
                return

            # Ici on force l'interprétation :
            # /scan voit uniquement devant avec un FOV de 200°
            # angle = -100° à gauche, +100° à droite, 0° devant
            half_fov_rad = math.radians(self.fov_deg / 2.0)
            angles = np.linspace(-half_fov_rad, half_fov_rad, n)

            valid_mask = np.isfinite(ranges) & (ranges > 0.0)

            front_mask = np.abs(angles) < math.radians(25.0)
            front_valid = ranges[valid_mask & front_mask]

            min_front = float(np.min(front_valid)) if front_valid.size > 0 else float('inf')

            influence_mask = valid_mask & (ranges < self.influence_distance)

            rep_x = 0.0
            rep_y = 0.0

            if np.any(influence_mask):
                r_infl = ranges[influence_mask]
                a_infl = angles[influence_mask]

                eps = 1e-6

                mags = self.k_repulsive * (
                    1.0 / (r_infl + eps) - 1.0 / self.influence_distance
                )

                mags = np.clip(mags, 0.0, None)

                # x = devant, y = gauche/droite
                fx = -mags * np.cos(a_infl)
                fy = -mags * np.sin(a_infl)

                rep_x = float(np.sum(fx))
                rep_y = float(np.sum(fy))

            # Attraction droit devant
            att_x = self.k_attractive
            att_y = 0.0

            total_x = att_x + rep_x
            total_y = att_y + rep_y

            resultant_angle = math.atan2(total_y, total_x)
            repulsive_mag = math.hypot(rep_x, rep_y)

            steering_angle = max(
                -self.max_steering_angle,
                min(self.max_steering_angle, resultant_angle)
            )

            steering_norm = steering_angle / self.max_steering_angle
            steering_norm = self.apply_steering_deadzone(repulsive_mag, steering_norm)
            steering_angle = steering_norm * self.max_steering_angle

            speed_cmd = self.max_speed

            if min_front < self.front_obstacle_threshold:
                if min_front <= self.critical_obstacle_distance:
                    obstacle_factor = 0.0
                else:
                    obstacle_factor = (
                        (min_front - self.critical_obstacle_distance)
                        / (self.front_obstacle_threshold - self.critical_obstacle_distance)
                    )
                    obstacle_factor = max(0.0, min(1.0, obstacle_factor))
                    obstacle_factor = obstacle_factor ** 0.9

                speed_cmd = self.min_speed + (
                    self.max_speed - self.min_speed
                ) * obstacle_factor

                speed_cmd *= 1.0 - 0.4 * abs(steering_norm)
            else:
                speed_cmd *= 1.0 - 0.2 * abs(steering_norm)

            speed_cmd = max(self.min_speed, min(self.max_speed, speed_cmd))

            self.smoothed_speed = (
                (1.0 - self.smoothing_alpha) * self.smoothed_speed
                + self.smoothing_alpha * speed_cmd
            )

            self.smoothed_steering = (
                (1.0 - self.smoothing_alpha) * self.smoothed_steering
                + self.smoothing_alpha * steering_angle
            )

            self.get_logger().info(
                f"PF 200deg -> front:{min_front:.2f}m, "
                f"rep_mag:{repulsive_mag:.3f}, "
                f"speed:{self.smoothed_speed:.3f}, "
                f"steer:{self.smoothed_steering:.3f}rad"
            )

            self.publish_ackermann(self.smoothed_speed, self.smoothed_steering)

        except Exception as e:
            self.get_logger().error(f"Error in scan_callback: {str(e)}")
            self.stop_vehicle()

    def emergency_callback(self, msg):
        self.emergency_stop = bool(msg.data)

        if self.emergency_stop:
            self.get_logger().warn("EMERGENCY STOP activated")
            self.stop_vehicle()
        else:
            self.get_logger().info("Emergency stop cleared")


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedPotentialFieldNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_vehicle()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()