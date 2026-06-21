#!/usr/bin/env python3

import math
import time as t
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive


class ImprovedFollowGapWithRecovery(Node):
    """
    Improved Follow The Gap pour circuit avec angles droits.

    Entrée :
      /scan                         sensor_msgs/LaserScan

    Sortie unique :
      /blue_car/cmd_ackermann      ackermann_msgs/AckermannDrive

    Principe :
      - FTG normal publie sur /blue_car/cmd_ackermann.
      - Si un mur est trop proche devant, on active une recovery non-bloquante.
      - Pendant la recovery, seul le mode recovery publie sur /blue_car/cmd_ackermann.
    """

    def __init__(self):
        super().__init__("improved_follow_gap_with_recovery")

        # ========================
        # Topics
        # ========================
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_topic", "/blue_car/cmd_ackermann")

        # ========================
        # FTG
        # ========================
        self.declare_parameter("max_steering", 0.28)
        self.declare_parameter("fov_deg", 200.0)
        self.declare_parameter("car_width", 0.20)

        self.declare_parameter("max_speed", 2.5)
        self.declare_parameter("min_speed", 0.5)

        self.declare_parameter("base_bubble", 0.20)
        self.declare_parameter("speed_bubble_gain", 0.08)

        self.declare_parameter("disparity_threshold", 0.40)
        self.declare_parameter("gap_edge_margin", 15)
        self.declare_parameter("free_distance_threshold", 0.30)

        self.declare_parameter("front_slow_distance", 2.5)
        self.declare_parameter("front_stop_distance", 0.35)

        # Lissage direction FTG
        # alpha proche de 0.0 = réactif ; alpha proche de 1.0 = très lissé.
        self.declare_parameter("steering_smoothing_alpha", 0.05)
        self.declare_parameter("max_steering_delta_per_scan", 0.35)

        # Lissage vitesse
        self.declare_parameter("speed_smoothing_alpha", 0.75)

        # ========================
        # Recovery
        # ========================
        self.declare_parameter("wall_too_close_distance", 0.32)
        self.declare_parameter("recovery_duration", 1.0)
        self.declare_parameter("recovery_cooldown", 0.30)

        self.declare_parameter("recovery_forward_speed", 0.9)
        self.declare_parameter("recovery_reverse_speed", -0.5)
        self.declare_parameter("recovery_forward_clear_distance", 0.35)
        self.declare_parameter("recovery_max_steering_angle", 0.28)
        self.declare_parameter("recovery_steering_gain", 1.0)
        self.declare_parameter("recovery_steering_smoothing_alpha", 0.35)
        self.declare_parameter("recovery_max_steering_delta_per_scan", 0.35)

        self.declare_parameter("recovery_front_center_index", -1)
        self.declare_parameter("recovery_invert_lidar_order", False)
        self.declare_parameter("recovery_invert_steering", False)
        self.declare_parameter("recovery_front_half_angle_deg", 15.0)
        self.declare_parameter("recovery_side_angle_deg", 105.0)

        # ========================
        # Debug
        # ========================
        self.declare_parameter("debug_logs", True)
        self.declare_parameter("debug_throttle_sec", 0.3)

        # ========================
        # Lecture paramètres
        # ========================
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)

        self.fov_deg = float(self.get_parameter("fov_deg").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.min_speed = float(self.get_parameter("min_speed").value)
        self.max_steering = float(self.get_parameter("max_steering").value)
        self.car_width = float(self.get_parameter("car_width").value)
        self.base_bubble = float(self.get_parameter("base_bubble").value)
        self.speed_bubble_gain = float(self.get_parameter("speed_bubble_gain").value)
        self.disparity_threshold = float(self.get_parameter("disparity_threshold").value)
        self.gap_edge_margin = int(self.get_parameter("gap_edge_margin").value)
        self.free_distance_threshold = float(self.get_parameter("free_distance_threshold").value)

        self.front_slow_distance = float(self.get_parameter("front_slow_distance").value)
        self.front_stop_distance = float(self.get_parameter("front_stop_distance").value)

        self.steering_smoothing_alpha = float(
            self.get_parameter("steering_smoothing_alpha").value
        )
        self.max_steering_delta_per_scan = float(
            self.get_parameter("max_steering_delta_per_scan").value
        )
        self.speed_smoothing_alpha = float(self.get_parameter("speed_smoothing_alpha").value)

        self.wall_too_close_distance = float(self.get_parameter("wall_too_close_distance").value)
        self.recovery_duration = float(self.get_parameter("recovery_duration").value)
        self.recovery_cooldown = float(self.get_parameter("recovery_cooldown").value)

        self.recovery_forward_speed = float(self.get_parameter("recovery_forward_speed").value)
        self.recovery_reverse_speed = float(self.get_parameter("recovery_reverse_speed").value)
        self.recovery_forward_clear_distance = float(
            self.get_parameter("recovery_forward_clear_distance").value
        )
        self.recovery_max_steering = float(
            self.get_parameter("recovery_max_steering_angle").value
        )
        self.recovery_steering_gain = float(self.get_parameter("recovery_steering_gain").value)
        self.recovery_steering_smoothing_alpha = float(
            self.get_parameter("recovery_steering_smoothing_alpha").value
        )
        self.recovery_max_steering_delta_per_scan = float(
            self.get_parameter("recovery_max_steering_delta_per_scan").value
        )

        self.recovery_front_center_index = int(
            self.get_parameter("recovery_front_center_index").value
        )
        self.recovery_invert_lidar_order = bool(
            self.get_parameter("recovery_invert_lidar_order").value
        )
        self.recovery_invert_steering = bool(
            self.get_parameter("recovery_invert_steering").value
        )
        self.recovery_front_half_angle = math.radians(
            float(self.get_parameter("recovery_front_half_angle_deg").value)
        )
        self.recovery_side_angle = math.radians(
            float(self.get_parameter("recovery_side_angle_deg").value)
        )

        self.debug_logs = bool(self.get_parameter("debug_logs").value)
        self.debug_throttle_sec = float(self.get_parameter("debug_throttle_sec").value)

        # ========================
        # État interne
        # ========================
        self.prev_steering = 0.0
        self.prev_recovery_steering = 0.0
        self.prev_speed = self.min_speed

        self.scan_points = None

        self.recovery_active = False
        self.recovery_end_time = 0.0
        self.last_recovery_time = -999.0

        # ========================
        # ROS interfaces
        # ========================
        self.sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10,
        )
        self.cmd_pub = self.create_publisher(AckermannDrive, self.cmd_topic, 10)

        self.get_logger().info(
            f"Improved FTG ready: scan={self.scan_topic}, cmd={self.cmd_topic}"
        )

    # ============================================================
    # Outils communs
    # ============================================================
    def publish_cmd(self, speed, steering):
        msg = AckermannDrive()
        msg.speed = float(speed)
        msg.steering_angle = float(steering)
        self.cmd_pub.publish(msg)

    def stop_cmd(self):
        self.publish_cmd(0.0, 0.0)

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def clean_range(self, r, range_max=6.0):
        if math.isnan(r) or math.isinf(r):
            return range_max
        if r <= 0.0:
            return 0.0
        return min(float(r), range_max)

    def smooth_value(self, target, previous, alpha, max_delta):
        """
        Filtre passe-bas + limite de variation.

        alpha = 0.0 : sortie = target, donc pas de lissage.
        alpha = 0.9 : sortie très proche de previous, donc très lissé.
        max_delta limite la variation absolue à chaque scan.
        """
        alpha = float(np.clip(alpha, 0.0, 1.0))
        max_delta = abs(float(max_delta))

        smoothed = alpha * previous + (1.0 - alpha) * target
        delta = smoothed - previous
        delta = float(np.clip(delta, -max_delta, max_delta))

        return previous + delta

    # ============================================================
    # Recovery lidar
    # ============================================================
    def update_recovery_scan_points(self, scan):
        ranges = [self.clean_range(r, scan.range_max) for r in scan.ranges]

        if self.recovery_invert_lidar_order:
            ranges = list(reversed(ranges))

        n = len(ranges)
        if n == 0:
            self.scan_points = None
            return

        front_angle_offset = 0.0
        if self.recovery_front_center_index >= 0:
            center = max(0, min(n - 1, self.recovery_front_center_index))
            front_angle_offset = scan.angle_min + center * scan.angle_increment

        points = []
        for i, r in enumerate(ranges):
            angle = scan.angle_min + i * scan.angle_increment - front_angle_offset
            points.append((self.normalize_angle(angle), r))

        self.scan_points = points

    def zone_mean(self, values):
        valid = [v for v in values if v > 0.0]
        if not valid:
            return 6.0
        return sum(valid) / len(valid)

    def zone_min(self, values):
        valid = [v for v in values if v > 0.0]
        if not valid:
            return 6.0
        return min(valid)

    def extract_recovery_zones(self):
        left = []
        front = []
        right = []

        if self.scan_points is None:
            return left, front, right

        for angle, distance in self.scan_points:
            if abs(angle) <= self.recovery_front_half_angle:
                front.append(distance)
            elif self.recovery_front_half_angle < angle <= self.recovery_side_angle:
                left.append(distance)
            elif -self.recovery_side_angle <= angle < -self.recovery_front_half_angle:
                right.append(distance)

        return left, front, right

    def compute_recovery_cmd(self):
        if self.scan_points is None or len(self.scan_points) < 9:
            return 0.0, 0.0, 0.0, "NO_SCAN"

        left, front, right = self.extract_recovery_zones()

        if not front:
            return 0.0, 0.0, 0.0, "NO_FRONT_ZONE"

        left_m = self.zone_mean(left)
        right_m = self.zone_mean(right)
        front_min = self.zone_min(front)

        # Si la droite est plus libre, right_m > left_m => steering négatif.
        steering_target = -self.recovery_steering_gain * (right_m - left_m) / max(0.20, front_min)
        steering_target = max(-1.0, min(1.0, steering_target)) * self.recovery_max_steering

        if self.recovery_invert_steering:
            steering_target = -steering_target

        if front_min >= self.recovery_forward_clear_distance:
            speed = self.recovery_forward_speed
            steering_target_for_motion = steering_target
            mode = "FORWARD"
        else:
            # Devant bloqué : marche arrière, avec braquage inversé.
            speed = self.recovery_reverse_speed
            steering_target_for_motion = -steering_target
            mode = "REVERSE"

        steering = self.smooth_value(
            steering_target_for_motion,
            self.prev_recovery_steering,
            self.recovery_steering_smoothing_alpha,
            self.recovery_max_steering_delta_per_scan,
        )
        self.prev_recovery_steering = steering

        return speed, steering, front_min, mode

    def start_recovery(self, now, front_min):
        self.recovery_active = True
        self.recovery_end_time = now + self.recovery_duration
        self.last_recovery_time = now
        self.prev_speed = 0.0
        self.prev_recovery_steering = self.prev_steering

        self.get_logger().warn(
            f"Mur trop proche devant: front_min={front_min:.2f} m < "
            f"{self.wall_too_close_distance:.2f} m. Recovery active."
        )

    def handle_recovery(self, now):
        if now >= self.recovery_end_time:
            self.recovery_active = False
            self.stop_cmd()
            self.prev_steering = self.prev_recovery_steering
            self.get_logger().info("Recovery finished, returning to FTG")
            return False

        speed, steering, front_min, mode = self.compute_recovery_cmd()
        self.publish_cmd(speed, steering)

        if self.debug_logs:
            self.get_logger().info(
                f"RECOVERY {mode} front={front_min:.2f} "
                f"speed={speed:.2f} steering={steering:.2f}",
                throttle_duration_sec=self.debug_throttle_sec,
            )
        return True

    # ============================================================
    # Improved Follow The Gap
    # ============================================================
    def preprocess_scan(self, scan):
        ranges = np.array(scan.ranges, dtype=np.float32)
        ranges[~np.isfinite(ranges)] = scan.range_max
        ranges = np.clip(ranges, scan.range_min, scan.range_max)

        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment

        half_fov = math.radians(self.fov_deg / 2.0)
        mask = np.logical_and(angles >= -half_fov, angles <= half_fov)

        return ranges[mask], angles[mask]

    def apply_disparity_extender(self, ranges, angles):
        ranges = ranges.copy()

        if len(ranges) < 2:
            return ranges

        angle_increment = abs(angles[1] - angles[0])
        if angle_increment <= 0.0:
            return ranges

        for i in range(len(ranges) - 1):
            d1 = float(ranges[i])
            d2 = float(ranges[i + 1])

            if abs(d1 - d2) < self.disparity_threshold:
                continue

            closer_dist = min(d1, d2)
            bubble_angle = math.atan2(self.car_width / 2.0, max(closer_dist, 0.05))
            bubble_count = int(bubble_angle / angle_increment)

            if d1 < d2:
                start = i + 1
                end = min(len(ranges), i + 1 + bubble_count)
            else:
                start = max(0, i - bubble_count)
                end = i

            ranges[start:end] = np.minimum(ranges[start:end], closer_dist)

        return ranges

    def apply_adaptive_bubble(self, ranges, angles):
        ranges = ranges.copy()

        if len(ranges) < 2:
            return ranges

        angle_increment = abs(angles[1] - angles[0])
        if angle_increment <= 0.0:
            return ranges

        min_index = int(np.argmin(ranges))
        min_dist = float(ranges[min_index])

        bubble_radius = self.base_bubble + self.speed_bubble_gain * abs(self.prev_speed)
        bubble_angle = math.atan2(bubble_radius, max(min_dist, 0.05))
        bubble_count = int(bubble_angle / angle_increment)

        start = max(0, min_index - bubble_count)
        end = min(len(ranges), min_index + bubble_count + 1)

        ranges[start:end] = 0.0
        return ranges

    def find_best_gap(self, ranges):
        free = ranges > self.free_distance_threshold

        best_start = 0
        best_end = 0
        current_start = None

        for i, is_free in enumerate(free):
            if is_free and current_start is None:
                current_start = i

            if (not is_free or i == len(free) - 1) and current_start is not None:
                current_end = i if not is_free else i + 1

                if current_end - current_start > best_end - best_start:
                    best_start = current_start
                    best_end = current_end

                current_start = None

        return best_start, best_end

    def choose_point_in_gap(self, ranges, angles, start, end):
        safe_start = start + self.gap_edge_margin
        safe_end = end - self.gap_edge_margin

        if safe_end <= safe_start:
            center_index = (start + end) // 2
            return max(0, min(len(ranges) - 1, center_index))

        gap_ranges = ranges[safe_start:safe_end]
        gap_angles = angles[safe_start:safe_end]

        # Score : distance libre, pas trop loin de l'avant, et pas de changement brutal.
        scores = (
            1.0 * gap_ranges
            - 1.0 * np.abs(gap_angles)
            - 0.7 * np.abs(gap_angles - self.prev_steering)
        )

        best_local = int(np.argmax(scores))
        return safe_start + best_local

    def compute_front_min(self, ranges, angles, half_angle_deg=12.0):
        front_mask = np.abs(angles) < math.radians(half_angle_deg)
        front_values = ranges[front_mask]

        if len(front_values) == 0:
            return 6.0

        valid = front_values[front_values > 0.0]
        if len(valid) == 0:
            return 6.0

        return float(np.min(valid))

    def compute_speed(self, ranges, angles, steering):
        front_min = self.compute_front_min(ranges, angles)

        if front_min < self.front_stop_distance:
            return 0.0

        denom = max(0.01, self.front_slow_distance - self.front_stop_distance)
        speed_ratio = (front_min - self.front_stop_distance) / denom
        speed_ratio = np.clip(speed_ratio, 0.0, 1.0)

        steering_penalty = 1.0 - min(abs(steering) / max(self.max_steering, 1e-6), 1.0)
        steering_penalty = 0.35 + 0.65 * steering_penalty

        speed = self.min_speed + speed_ratio * (self.max_speed - self.min_speed)
        speed *= steering_penalty

        return float(np.clip(speed, 0.0, self.max_speed))

    # ============================================================
    # Callback principal
    # ============================================================
    def scan_callback(self, scan):
        self.update_recovery_scan_points(scan)
        now = t.monotonic()

        # Si recovery en cours, elle garde la main et publie sur le même topic.
        if self.recovery_active:
            still_active = self.handle_recovery(now)
            if still_active:
                return
            # Si elle vient de finir, on continue avec FTG sur ce scan.

        ranges, angles = self.preprocess_scan(scan)

        if len(ranges) < 10:
            self.stop_cmd()
            return

        raw_front_min = self.compute_front_min(ranges, angles, half_angle_deg=15.0)

        # Déclenchement recovery.
        if (
            raw_front_min < self.wall_too_close_distance
            and (now - self.last_recovery_time) > self.recovery_cooldown
        ):
            self.start_recovery(now, raw_front_min)
            self.handle_recovery(now)
            return

        # FTG normal.
        ftg_ranges = self.apply_disparity_extender(ranges, angles)
        ftg_ranges = self.apply_adaptive_bubble(ftg_ranges, angles)

        gap_start, gap_end = self.find_best_gap(ftg_ranges)

        if gap_end <= gap_start:
            self.stop_cmd()
            return

        best_index = self.choose_point_in_gap(ftg_ranges, angles, gap_start, gap_end)
        target_angle = float(angles[best_index])

        steering_target = float(np.clip(target_angle, -self.max_steering, self.max_steering))
        steering = self.smooth_value(
            steering_target,
            self.prev_steering,
            self.steering_smoothing_alpha,
            self.max_steering_delta_per_scan,
        )

        speed_target = self.compute_speed(ranges, angles, steering)
        speed = self.smooth_value(
            speed_target,
            self.prev_speed,
            self.speed_smoothing_alpha,
            max_delta=self.max_speed,
        )

        self.prev_steering = steering
        self.prev_speed = speed

        if self.debug_logs:
            self.get_logger().info(
                f"FTG front={raw_front_min:.2f} "
                f"speed={speed:.2f} "
                f"steering={steering:.2f} "
                f"target={steering_target:.2f} "
                f"gap=({gap_start},{gap_end}) "
                f"target_angle={math.degrees(target_angle):.1f}deg",
                throttle_duration_sec=self.debug_throttle_sec,
            )

        self.publish_cmd(speed, steering)


def main(args=None):
    rclpy.init(args=args)
    node = ImprovedFollowGapWithRecovery()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_cmd()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
