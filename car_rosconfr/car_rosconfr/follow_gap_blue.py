from math import isnan, isinf, degrees
import sys
import select
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive


class FollowGap(Node):
    def __init__(self):
        super().__init__("follow_gap")

        self.pub_cmd = self.create_publisher(
            AckermannDrive,
            "/blue_car/cmd_ackermann",
            10
        )

        self.sub_scan = self.create_subscription(
            LaserScan,
            "/blue_car/scan",
            self.scan_callback,
            10
        )

        # --- Calibration lidar ---
        self.declare_parameter("front_center_index", 180)
        self.declare_parameter("invert_lidar_order", True)

        self.front_center_index = int(
            self.get_parameter("front_center_index").value
        )

        self.invert_lidar_order = bool(
            self.get_parameter("invert_lidar_order").value
        )

        # --- Paramètres physiques ---
        self.turning_radius = 0.90
        self.vehicle_width  = 0.20

        # --- Bulle adaptative ---
        self.bubble_angle_min_deg = 15.0
        self.bubble_angle_max_deg = 40.0
        self.bubble_dist_max      = 3.0

        self.max_range                = 6.0
        self.min_valid_range          = 0.1
        self.front_half_angle_deg     = 80.0
        self.free_space_threshold     = 1.8
        self.emergency_stop_dist      = 0.25
        self.emergency_half_angle_deg = 25.0

        # --- Vitesses ---
        self.speed_straight = 1.6 #1.0
        self.speed_medium   = 1.2 #0.8
        self.speed_turn     = 1.0 #0.7

        # --- Lissage ---
        self.alpha         = 0.50
        self.alpha_free    = 0.30
        self.last_cmd_dir  = 0.0

        # --- Recovery après arrêt d'urgence ---
        self.recovery_mode = False
        self.recovery_steps_remaining = 0
        self.recovery_steps_total = 18
        self.recovery_speed = -0.50
        self.recovery_dir_gain = 0.8
        self.recovery_cooldown = 0
        self.recovery_cooldown_total = 6

        self.angle_increment = None
        self.logged_info     = False

        # --- Stop clavier ---
        self.stop_requested = False
        self._keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self._keyboard_thread.start()

        self.get_logger().info("Follow Gap (SLAM/Covapsy mode) started")
        self.get_logger().info("Appuie sur 'q' dans ce terminal pour arrêter la voiture.")

    # ------------------------------------------------------------------
    def keyboard_listener(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key.lower() == 'q':
                        self.stop_requested = True
                        self.publish_cmd(0.0, 0.0)
                        self.get_logger().warn("Touche 'q' détectée -> STOP véhicule")
                        break
        except Exception as e:
            self.get_logger().error(f"Erreur keyboard listener: {e}")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    # ------------------------------------------------------------------
    def _update_geometry(self, scan_msg):
        angle_increment_rad = scan_msg.angle_increment

        if angle_increment_rad is None or angle_increment_rad <= 0.0:
            n = len(scan_msg.ranges)

            if n > 1 and scan_msg.angle_max > scan_msg.angle_min:
                angle_increment_rad = (scan_msg.angle_max - scan_msg.angle_min) / (n - 1)
            elif n > 0:
                # fallback pour un lidar 360 points sur 360°
                angle_increment_rad = 2.0 * 3.141592653589793 / n
            else:
                return

        self.angle_increment = angle_increment_rad

        self.get_logger().info(
            f"angle_increment={degrees(angle_increment_rad):.4f}° "
            f"| bubble_angle [{self.bubble_angle_min_deg}°, {self.bubble_angle_max_deg}°]"
        )

    # ------------------------------------------------------------------
    def _adaptive_bubble_radius(self, obstacle_dist):
        """
        Bulle proportionnelle à la proximité de l'obstacle.
        """
        t = max(
            0.0,
            min(
                1.0,
                1.0 - (obstacle_dist - self.emergency_stop_dist)
                / (self.bubble_dist_max - self.emergency_stop_dist)
            )
        )
        angle_deg = self.bubble_angle_min_deg + t * (
            self.bubble_angle_max_deg - self.bubble_angle_min_deg
        )
        pts_per_deg = 1.0 / degrees(self.angle_increment)
        radius = int(angle_deg * pts_per_deg)
        return radius

    # ------------------------------------------------------------------
    def preprocess(self, ranges):
        out = []
        for r in ranges:
            if isnan(r) or isinf(r):
                out.append(self.max_range)
            elif r < self.min_valid_range:
                out.append(0.0)
            else:
                out.append(min(r, self.max_range))
        return out

    # ------------------------------------------------------------------
    def find_largest_gap(self, arr):
        """
        Score = largeur × profondeur moyenne.
        Les gaps trop étroits pour le véhicule sont ignorés.
        """
        min_gap_width = max(
            1,
            int((self.vehicle_width / self.turning_radius) / degrees(self.angle_increment))
        )

        best_start, best_end = 0, -1
        best_score = -1.0
        start = None

        for i, v in enumerate(arr):
            if v > 0.0 and start is None:
                start = i
            elif (v == 0.0 or i == len(arr) - 1) and start is not None:
                end = i - 1 if v == 0.0 else i
                width = end - start + 1
                if width >= min_gap_width:
                    depth = sum(arr[start:end + 1]) / width
                    score = width * depth
                    if score > best_score:
                        best_score = score
                        best_start, best_end = start, end
                start = None

        if start is not None:
            end = len(arr) - 1
            width = end - start + 1
            if width >= min_gap_width:
                depth = sum(arr[start:end + 1]) / width
                score = width * depth
                if score > best_score:
                    best_start, best_end = start, end

        return best_start, best_end

    # ------------------------------------------------------------------
    def _best_point_in_gap(self, arr, gap_start, gap_end):
        mid = (gap_start + gap_end) // 2
        best_i = mid
        best_score = -1e9

        for i in range(gap_start, gap_end + 1):
            depth = arr[i]
            center_penalty = abs(i - mid) * 0.03
            score = depth - center_penalty
            if score > best_score:
                best_score = score
                best_i = i

        return best_i

    # ------------------------------------------------------------------
    def start_recovery(self):
        self.recovery_mode = True
        self.recovery_steps_remaining = self.recovery_steps_total
        self.recovery_cooldown = self.recovery_cooldown_total

    # ------------------------------------------------------------------
    def publish_cmd(self, direction, speed):
        msg = AckermannDrive()

        # direction est entre -1 et 1
        # on la convertit en angle de braquage réel
        max_steering_angle = 0.28  # rad, cohérent avec TT02

        msg.steering_angle = max(-1.0, min(1.0, float(direction))) * max_steering_angle
        msg.speed = float(speed)

        self.pub_cmd.publish(msg)

    # ------------------------------------------------------------------
    def get_front_window(self, ranges):
        n = len(ranges)
        if n == 0:
            return [], 0, 0

        if self.invert_lidar_order:
            ranges = list(reversed(ranges))

        center = self.front_center_index

        # sécurité si l'indice est hors limites
        center = max(0, min(n - 1, center))

        half_window = int((self.front_half_angle_deg / 360.0) * n)

        start_i = center - half_window
        end_i = center + half_window

        # Gestion circulaire du lidar
        if start_i < 0:
            front = ranges[start_i:] + ranges[:end_i + 1]
        elif end_i >= n:
            front = ranges[start_i:] + ranges[:end_i - n + 1]
        else:
            front = ranges[start_i:end_i + 1]

        center_i = len(front) // 2

        return front, center_i, half_window

    # ------------------------------------------------------------------
    def scan_callback(self, scan_msg):
        # Stop demandé au clavier
        if self.stop_requested:
            self.publish_cmd(0.0, 0.0)
            return

        if self.angle_increment is None:
            self._update_geometry(scan_msg)

        if self.angle_increment is None:
            self.get_logger().warn("angle_increment invalide -> scan ignoré")
            return

        # --- Manoeuvre de recul en cours ---
        if self.recovery_mode:
            if abs(self.last_cmd_dir) < 0.1:
                recovery_dir = -0.7
            else:
                recovery_dir = -self.recovery_dir_gain * self.last_cmd_dir

            recovery_dir = max(-1.0, min(1.0, recovery_dir))
            self.publish_cmd(recovery_dir, self.recovery_speed)

            self.recovery_steps_remaining -= 1
            self.get_logger().warn(
                f"RECOVERY: reverse dir={recovery_dir:.2f} "
                f"steps_left={self.recovery_steps_remaining}"
            )

            if self.recovery_steps_remaining <= 0:
                self.recovery_mode = False

            return

        ranges = self.preprocess(list(scan_msg.ranges))
        n = len(ranges)
        if n == 0:
            return

        # half_window = int((self.front_half_angle_deg / 360.0) * n)
        # front = ranges[n - half_window:] + ranges[:half_window + 1]

        front, center_i, half_window = self.get_front_window(ranges)

        if len(front) == 0:
            return

        center_i = len(front) // 2

        if not self.logged_info:
            self.logged_info = True
            self.get_logger().info(
                f"n={n}, front_center_index={self.front_center_index}, "
                f"invert_lidar_order={self.invert_lidar_order}, "
                f"half_window={half_window}, front_len={len(front)}, center_i={center_i}"
            )

        valid = [r for r in front if r > 0.0]
        if len(valid) == 0:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().warn("Scan entièrement masqué -> STOP")
            return

        # Zone étroite uniquement pour le stop d'urgence
        em_half = int((self.emergency_half_angle_deg / self.front_half_angle_deg) * center_i)
        em_start = max(0, center_i - em_half)
        em_end   = min(len(front), center_i + em_half + 1)

        emergency_zone = [r for r in front[em_start:em_end] if r > 0.0]
        if len(emergency_zone) == 0:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().warn("Zone d'urgence vide -> STOP")
            return

        min_r_front = min(valid)
        min_r_emergency = min(emergency_zone)

        if self.recovery_cooldown > 0:
            self.recovery_cooldown -= 1

        # Arrêt d'urgence
        if min_r_emergency < self.emergency_stop_dist:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().warn(
                f"EMERGENCY STOP: obstacle à {min_r_emergency:.2f} m"
            )

            if self.recovery_cooldown == 0:
                self.start_recovery()

            return

        # ---- Espace libre : gap centré, lissage modéré ----
        if min_r_front > self.free_space_threshold:
            gap_start, gap_end = self.find_largest_gap(front)
            if gap_end >= gap_start:
                best_i = self._best_point_in_gap(front, gap_start, gap_end)
                error = best_i - center_i
                cmd_dir = max(-1.0, min(1.0, (error * 2.0) / center_i))
            else:
                cmd_dir = 0.0

            cmd_dir = self.alpha_free * cmd_dir + (1.0 - self.alpha_free) * self.last_cmd_dir
            self.last_cmd_dir = cmd_dir
            self.publish_cmd(cmd_dir, self.speed_straight)
            return

        # ---- Obstacle détecté : bulle adaptative ----
        closest_i = min(
            range(len(front)),
            key=lambda i: front[i] if front[i] > 0.0 else 1e9
        )
        obstacle_dist = front[closest_i]
        adaptive_r = self._adaptive_bubble_radius(obstacle_dist)

        b_start = max(0, closest_i - adaptive_r)
        b_end = min(len(front), closest_i + adaptive_r + 1)
        for i in range(b_start, b_end):
            front[i] = 0.0

        gap_start, gap_end = self.find_largest_gap(front)
        if gap_end < gap_start:
            self.publish_cmd(0.0, self.speed_turn)
            self.get_logger().warn("Aucun gap valide trouvé")
            return

        best_i = self._best_point_in_gap(front, gap_start, gap_end)
        error = best_i - center_i
        cmd_dir = max(-1.0, min(1.0, (error * 2.0) / center_i))

        cmd_dir = self.alpha * cmd_dir + (1.0 - self.alpha) * self.last_cmd_dir
        self.last_cmd_dir = cmd_dir

        abs_dir = abs(cmd_dir)
        if abs_dir < 0.2:
            cmd_speed = self.speed_straight
        elif abs_dir < 0.5:
            cmd_speed = self.speed_medium
        else:
            cmd_speed = self.speed_turn

        self.get_logger().info(
            f"min_front={min_r_front:.2f}m | min_em={min_r_emergency:.2f}m | "
            f"bubble_r={adaptive_r}pts | gap=({gap_start},{gap_end}) | "
            f"best={best_i} | dir={cmd_dir:.3f} | spd={cmd_speed:.2f}"
        )

        self.publish_cmd(cmd_dir, cmd_speed)


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = FollowGap()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
