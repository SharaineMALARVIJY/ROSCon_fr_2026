#!/usr/bin/env python3

import math
from statistics import mean, median

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive


class SimpleReactiveWithRecovery(Node):
    """
    Controller ROS2 reactif simple avec recovery marche arriere.

    Logique normale volontairement simple:
      - lire deux fenetres lidar gauche/droite
      - calculer error = right_dist - left_dist
      - steering = kp_steer * error
      - vitesse quasi constante

    Recovery:
      - si obstacle trop proche devant: marche arriere pendant quelques steps
      - pas de verification arriere, car le lidar ne voit que l'avant
      - braquage de recul choisi avec le scan avant ou l'ancien braquage
    """

    def __init__(self):
        super().__init__("simple_reactive_with_recovery_node")

        # Topics
        self.declare_parameter("scan_topic", "/blue_car/scan")
        self.declare_parameter("cmd_topic", "/blue_car/cmd_ackermann")
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)

        # Calibration / sens du lidar
        # Si ton scan ROS est deja en angles corrects, laisse use_angle_windows=True.
        # Angles en degres, avec 0 deg devant, + a gauche dans ROS standard.
        self.declare_parameter("use_angle_windows", True)
        self.declare_parameter("invert_steering", False)
        self.declare_parameter("invert_error", True)

        self.use_angle_windows = bool(self.get_parameter("use_angle_windows").value)
        self.invert_steering = bool(self.get_parameter("invert_steering").value)
        self.invert_error = bool(self.get_parameter("invert_error").value)

        # Fenetres gauche/droite utilisees pour le controle reactif.
        # Pour un lidar avant 200 deg, ces fenetres correspondent grossierement
        # a des zones laterales avant. Elles restent parametrables.
        self.declare_parameter("left_angle_min_deg", 50.0)
        self.declare_parameter("left_angle_max_deg", 70.0)
        self.declare_parameter("right_angle_min_deg", -70.0)
        self.declare_parameter("right_angle_max_deg", -50.0)

        self.left_angle_min = math.radians(float(self.get_parameter("left_angle_min_deg").value))
        self.left_angle_max = math.radians(float(self.get_parameter("left_angle_max_deg").value))
        self.right_angle_min = math.radians(float(self.get_parameter("right_angle_min_deg").value))
        self.right_angle_max = math.radians(float(self.get_parameter("right_angle_max_deg").value))

        # Fallback par indices si besoin.
        self.declare_parameter("left_index_min", 110)
        self.declare_parameter("left_index_max", 130)
        self.declare_parameter("right_index_min", 230)
        self.declare_parameter("right_index_max", 250)

        self.left_index_min = int(self.get_parameter("left_index_min").value)
        self.left_index_max = int(self.get_parameter("left_index_max").value)
        self.right_index_min = int(self.get_parameter("right_index_min").value)
        self.right_index_max = int(self.get_parameter("right_index_max").value)

        # Controle normal
        self.declare_parameter("speed", 1.8)  # 1.5 m/s ~= 5.4 km/h
        self.declare_parameter("kp_steer", 0.55)
        self.declare_parameter("max_steering_angle", 0.28)
        self.declare_parameter("feature", "mean")  # mean ou median
        self.declare_parameter("max_lidar_range", 12.0)

        self.speed = float(self.get_parameter("speed").value)
        self.kp_steer = float(self.get_parameter("kp_steer").value)
        self.max_steering_angle = float(self.get_parameter("max_steering_angle").value)
        self.feature = str(self.get_parameter("feature").value)
        self.max_lidar_range = float(self.get_parameter("max_lidar_range").value)

        # Pondération de vitesse par un rayon/cone avant.
        # La direction reste celle du controller simple, seule la vitesse est modulée.
        self.declare_parameter("enable_front_speed_weight", True)
        self.declare_parameter("front_speed_angle_deg", 10.0)  # cone avant: [-10 deg, +10 deg]
        self.declare_parameter("front_slow_distance", 0.25)    # en dessous: vitesse minimale
        self.declare_parameter("front_full_speed_distance", 1.20)  # au dessus: vitesse nominale
        self.declare_parameter("min_speed_ratio", 0.35)        # 0.35 * speed
        self.declare_parameter("front_speed_feature", "min")  # min, mean ou median

        self.enable_front_speed_weight = bool(
            self.get_parameter("enable_front_speed_weight").value
        )
        self.front_speed_angle = math.radians(
            float(self.get_parameter("front_speed_angle_deg").value)
        )
        self.front_slow_distance = float(self.get_parameter("front_slow_distance").value)
        self.front_full_speed_distance = float(
            self.get_parameter("front_full_speed_distance").value
        )
        self.min_speed_ratio = float(self.get_parameter("min_speed_ratio").value)
        self.front_speed_feature = str(self.get_parameter("front_speed_feature").value)

        # Filtrage leger: 1.0 = aucun filtrage, proche simple.
        self.declare_parameter("alpha_steering", 0.85)
        self.alpha_steering = float(self.get_parameter("alpha_steering").value)
        self.last_steering = 0.0
        self.last_speed = 0.0

        # Recovery marche arriere, repris de dial_nav_blue en version minimale.
        self.declare_parameter("enable_reverse", True)
        self.declare_parameter("emergency_distance", 0.35)
        self.declare_parameter("reverse_release_distance", 0.55)
        self.declare_parameter("reverse_speed", -0.35)
        self.declare_parameter("reverse_steps", 25)
        self.declare_parameter("min_reverse_steps", 8)
        self.declare_parameter("reverse_stop_steps", 3)
        self.declare_parameter("reverse_cooldown_steps", 8)
        self.declare_parameter("reverse_steering_mode", "auto")  # auto, last_opposite, straight
        self.declare_parameter("reverse_steering_gain", 0.9)
        self.declare_parameter("front_emergency_angle_deg", 12.0)

        self.enable_reverse = bool(self.get_parameter("enable_reverse").value)
        self.emergency_distance = float(self.get_parameter("emergency_distance").value)
        self.reverse_release_distance = float(self.get_parameter("reverse_release_distance").value)
        self.reverse_speed = float(self.get_parameter("reverse_speed").value)
        self.reverse_steps = int(self.get_parameter("reverse_steps").value)
        self.min_reverse_steps = int(self.get_parameter("min_reverse_steps").value)
        self.reverse_stop_steps = int(self.get_parameter("reverse_stop_steps").value)
        self.reverse_cooldown_steps = int(self.get_parameter("reverse_cooldown_steps").value)
        self.reverse_steering_mode = str(self.get_parameter("reverse_steering_mode").value)
        self.reverse_steering_gain = float(self.get_parameter("reverse_steering_gain").value)
        self.front_emergency_angle = math.radians(float(self.get_parameter("front_emergency_angle_deg").value))

        self.reverse_remaining = 0
        self.reverse_done_steps = 0
        self.reverse_steering = 0.0
        self.reverse_stop_remaining = 0
        self.reverse_cooldown_remaining = 0

        self.cmd_pub = self.create_publisher(AckermannDrive, self.cmd_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        self.get_logger().info("Simple reactive with recovery demarre")
        self.get_logger().info(f"scan_topic={self.scan_topic}, cmd_topic={self.cmd_topic}")
        self.get_logger().info(
            "normal: steering = kp_steer * (right_dist - left_dist), "
            f"speed={self.speed:.2f}, kp={self.kp_steer:.2f}"
        )

    def crop(self, value, vmin, vmax):
        return max(vmin, min(vmax, value))

    def valid_range(self, r):
        return not math.isnan(r) and not math.isinf(r) and r > 0.0

    def clean_range(self, r):
        if math.isnan(r) or math.isinf(r):
            return self.max_lidar_range
        if r <= 0.0:
            return 0.0
        return min(float(r), self.max_lidar_range)

    def feature_value(self, values, default=None):
        valid = [self.clean_range(v) for v in values if self.valid_range(v)]
        if not valid:
            return default
        if self.feature == "median":
            return float(median(valid))
        return float(mean(valid))

    def angle_of_index(self, msg, i):
        return msg.angle_min + i * msg.angle_increment

    def values_in_angle_window(self, msg, amin, amax):
        lo = min(amin, amax)
        hi = max(amin, amax)
        out = []
        for i, r in enumerate(msg.ranges):
            a = self.angle_of_index(msg, i)
            if lo <= a <= hi:
                out.append(r)
        return out

    def values_in_index_window(self, msg, imin, imax):
        n = len(msg.ranges)
        imin = max(0, min(n - 1, int(imin)))
        imax = max(0, min(n - 1, int(imax)))
        if imax < imin:
            imin, imax = imax, imin
        return list(msg.ranges[imin:imax + 1])

    def front_distances(self, msg):
        """Distance minimale devant + devant gauche/droite pour choisir le recul."""
        front = []
        front_left = []
        front_right = []

        for i, r in enumerate(msg.ranges):
            if not self.valid_range(r):
                continue
            a = self.angle_of_index(msg, i)
            if abs(a) <= self.front_emergency_angle:
                front.append(r)
                if a >= 0.0:
                    front_left.append(r)
                else:
                    front_right.append(r)

        def safe_min(vals):
            return min(vals) if vals else 999.0

        return {
            "front": safe_min(front),
            "front_left": safe_min(front_left),
            "front_right": safe_min(front_right),
        }

    def front_speed_distance(self, msg):
        """Distance avant utilisée uniquement pour pondérer la vitesse."""
        values = []
        for i, r in enumerate(msg.ranges):
            if not self.valid_range(r):
                continue
            a = self.angle_of_index(msg, i)
            if abs(a) <= self.front_speed_angle:
                values.append(self.clean_range(r))

        if not values:
            return self.max_lidar_range

        if self.front_speed_feature == "mean":
            return float(mean(values))
        if self.front_speed_feature == "median":
            return float(median(values))
        return float(min(values))

    def compute_weighted_speed(self, msg):
        """Retourne speed pondérée par l'espace libre devant."""
        if not self.enable_front_speed_weight:
            return self.speed, self.max_lidar_range, 1.0

        front_dist = self.front_speed_distance(msg)

        # Rampe linéaire:
        # - front_slow_distance -> min_speed_ratio * speed
        # - front_full_speed_distance -> speed nominale
        denom = max(0.001, self.front_full_speed_distance - self.front_slow_distance)
        ratio = (front_dist - self.front_slow_distance) / denom
        ratio = self.crop(ratio, self.min_speed_ratio, 1.0)

        return self.speed * ratio, front_dist, ratio

    def publish_raw_cmd(self, speed, steering):
        msg = AckermannDrive()
        msg.speed = float(speed)
        msg.steering_angle = float(steering)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.last_speed = 0.0
        self.last_steering = 0.0
        self.publish_raw_cmd(0.0, 0.0)

    def choose_reverse_steering(self, distances):
        if self.reverse_steering_mode == "straight":
            steering = 0.0
        elif self.reverse_steering_mode == "last_opposite":
            steering = -self.last_steering
        else:
            # Si l'avant droit est plus libre, braquage positif; sinon negatif.
            direction = 1.0 if distances["front_right"] >= distances["front_left"] else -1.0
            steering = direction * self.reverse_steering_gain * self.max_steering_angle

        steering = self.crop(steering, -self.max_steering_angle, self.max_steering_angle)
        if self.invert_steering:
            steering = -steering
        return steering

    def reverse_step(self):
        self.last_speed = self.reverse_speed
        self.last_steering = self.reverse_steering
        self.publish_raw_cmd(self.reverse_speed, self.reverse_steering)

    def start_reverse(self, distances):
        self.reverse_remaining = self.reverse_steps
        self.reverse_done_steps = 0
        self.reverse_steering = self.choose_reverse_steering(distances)
        self.get_logger().warn(
            f"Obstacle avant {distances['front']:.2f} m -> reverse "
            f"steer={self.reverse_steering:.2f} "
            f"FL={distances['front_left']:.2f} FR={distances['front_right']:.2f}"
        )
        self.reverse_step()

    def finish_reverse(self):
        self.reverse_remaining = 0
        self.reverse_done_steps = 0
        self.reverse_steering = 0.0
        self.reverse_stop_remaining = self.reverse_stop_steps
        self.reverse_cooldown_remaining = self.reverse_cooldown_steps
        self.stop()

    def normal_control(self, msg):
        if self.use_angle_windows:
            left_values = self.values_in_angle_window(msg, self.left_angle_min, self.left_angle_max)
            right_values = self.values_in_angle_window(msg, self.right_angle_min, self.right_angle_max)
        else:
            left_values = self.values_in_index_window(msg, self.left_index_min, self.left_index_max)
            right_values = self.values_in_index_window(msg, self.right_index_min, self.right_index_max)

        left_dist = self.feature_value(left_values, default=None)
        right_dist = self.feature_value(right_values, default=None)

        # Meme fallback que l'esprit simple: eviter un coup brutal si le lidar est mauvais.
        if left_dist is None or right_dist is None:
            steering = 0.8 * self.last_steering
            self.get_logger().warn("Fenetre lidar invalide -> conservation amortie du steering")
        else:
            error = right_dist - left_dist
            if self.invert_error:
                error = -error
            steering = self.kp_steer * error
            self.get_logger().info(
                f"L={left_dist:.2f} R={right_dist:.2f} "
                f"err={error:.2f} steer_raw={steering:.2f}"
            )

        steering = self.crop(steering, -self.max_steering_angle, self.max_steering_angle)
        if self.invert_steering:
            steering = -steering

        # Filtrage leger, desactivable avec alpha_steering=1.0.
        steering = (1.0 - self.alpha_steering) * self.last_steering + self.alpha_steering * steering
        steering = self.crop(steering, -self.max_steering_angle, self.max_steering_angle)

        speed_cmd, front_speed_dist, speed_ratio = self.compute_weighted_speed(msg)

        self.get_logger().info(
            f"front_speed={front_speed_dist:.2f} ratio={speed_ratio:.2f} "
            f"speed={speed_cmd:.2f} steer={steering:.2f}"
        )

        self.last_speed = speed_cmd
        self.last_steering = steering
        self.publish_raw_cmd(speed_cmd, steering)

    def scan_callback(self, msg):
        if len(msg.ranges) == 0:
            self.stop()
            return

        distances = self.front_distances(msg)
        min_front = distances["front"]

        # Pause apres un recul.
        if self.enable_reverse and self.reverse_stop_remaining > 0:
            self.reverse_stop_remaining -= 1
            self.stop()
            return

        if self.reverse_cooldown_remaining > 0:
            self.reverse_cooldown_remaining -= 1

        # Marche arriere en cours.
        if self.enable_reverse and self.reverse_remaining > 0:
            self.reverse_remaining -= 1
            self.reverse_done_steps += 1

            reverse_can_stop = self.reverse_done_steps >= self.min_reverse_steps
            front_is_clear = min_front > self.reverse_release_distance

            if reverse_can_stop and front_is_clear:
                self.get_logger().warn(
                    f"Fin reverse: front={min_front:.2f} m apres {self.reverse_done_steps} steps"
                )
                self.finish_reverse()
                return

            self.reverse_step()
            return

        # Declenchement recovery.
        can_start_reverse = self.reverse_cooldown_remaining == 0
        if self.enable_reverse and can_start_reverse and min_front < self.emergency_distance:
            self.start_reverse(distances)
            return

        # Si cooldown actif mais obstacle encore colle devant: stop plutot que repartir dedans.
        if self.enable_reverse and min_front < self.emergency_distance:
            self.stop()
            return

        self.normal_control(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleReactiveWithRecovery()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
