#!/usr/bin/env python3
"""
ROS 2 node: camera red/green direction detector + LiDAR-assisted recovery for Ackermann car.

Goal:
  - Detect whether the car is facing the wrong way using red/green walls in camera image.
  - If wrong direction is detected, take temporary control and perform repeated forward/reverse
    manoeuvres until the camera clearly sees the correct red/green ordering.
  - Use LiDAR to avoid driving into walls and choose the safest steering direction.

Default topics for yellow_car / Webots:
  Input camera: /yellow_car/image_raw/image_color
  Input LiDAR:  /scan
  Output cmd:   /yellow_car/cmd_ackermann_direction_recovery
  Output enable:/yellow_car/direction_recovery_enable
  Debug image:  /yellow_car/color_direction_lidar_debug

Expected convention:
  expected_green_side = "left" means: when the car is in the correct direction,
  the green wall should appear mostly on the LEFT side of the camera image.

Notes:
  - This node is intended to publish on a PRIORITY Ackermann topic connected to your mux.
  - It publishes a Bool enable topic so the mux knows explicitly when this recovery controls.
  - Ackermann zero commands are allowed while enable=True: they mean controlled stop.
  - During one recovery attempt, the steering direction is frozen by default so the
    forward phase does not undo the reverse phase.
  - As soon as the camera sees GOOD, the car stops immediately and waits for confirmation
    before releasing enable.
"""

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Bool


@dataclass
class DirectionState:
    state: str = "UNKNOWN"  # GOOD / WRONG / UNKNOWN
    green_side: str = "unknown"  # left / right / center / unknown
    red_side: str = "unknown"
    green_area: int = 0
    red_area: int = 0
    confidence: float = 0.0


@dataclass
class LidarState:
    front_min: float = float("inf")
    rear_min: float = float("inf")
    left_min: float = float("inf")
    right_min: float = float("inf")
    left_front_min: float = float("inf")
    right_front_min: float = float("inf")
    left_rear_min: float = float("inf")
    right_rear_min: float = float("inf")
    best_turn: int = 1  # +1 left, -1 right
    stamp: float = 0.0


class ColorDirectionLidarRecovery(Node):
    def __init__(self):
        super().__init__("color_direction_lidar_recovery")

        # Topics
        self.declare_parameter("image_topic", "/yellow_car/image_raw/image_color")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_topic", "/yellow_car/cmd_ackermann_direction_recovery")
        self.declare_parameter("enable_topic", "/yellow_car/direction_recovery_enable")
        self.declare_parameter("debug_image_topic", "/yellow_car/color_direction_lidar_debug")

        # Direction convention
        self.declare_parameter("expected_green_side", "right")  # left or right

        # Image processing
        self.declare_parameter("use_lower_half_only", True)
        self.declare_parameter("min_color_area", 450)
        self.declare_parameter("min_area_ratio", 0.0015)
        self.declare_parameter("side_deadband_px", 35)
        self.declare_parameter("wrong_frames_to_trigger", 5)
        self.declare_parameter("good_frames_to_release", 8)
        self.declare_parameter("unknown_frames_before_recovery", 20)

        # Recovery movement
        self.declare_parameter("reverse_speed", -0.6)
        self.declare_parameter("forward_speed", 1.0)
        self.declare_parameter("max_steering", 0.28)
        self.declare_parameter("reverse_duration", 1.1)
        self.declare_parameter("forward_duration", 1.1)
        self.declare_parameter("stop_duration", 0.20)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("max_recovery_duration", 25.0)  # 0 disables timeout

        # LiDAR safety
        self.declare_parameter("front_stop_dist", 0.42)
        self.declare_parameter("rear_stop_dist", 0.35)
        self.declare_parameter("side_stop_dist", 0.22)
        self.declare_parameter("front_sector_deg", 35.0)
        self.declare_parameter("rear_sector_deg", 35.0)
        self.declare_parameter("side_sector_deg", 55.0)
        self.declare_parameter("scan_timeout", 0.50)
        self.declare_parameter("use_lidar_turn_choice", True)
        self.declare_parameter("freeze_turn_direction_during_recovery", True)
        self.declare_parameter("stop_as_soon_as_good_seen", True)

        # Debug/logging
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("log_period", 0.8)

        self.image_topic = self.get_parameter("image_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.enable_topic = self.get_parameter("enable_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value

        self.expected_green_side = str(self.get_parameter("expected_green_side").value).lower()
        if self.expected_green_side not in ("left", "right"):
            self.get_logger().warn("expected_green_side must be 'left' or 'right'. Falling back to 'left'.")
            self.expected_green_side = "left"

        self.use_lower_half_only = bool(self.get_parameter("use_lower_half_only").value)
        self.min_color_area = int(self.get_parameter("min_color_area").value)
        self.min_area_ratio = float(self.get_parameter("min_area_ratio").value)
        self.side_deadband_px = int(self.get_parameter("side_deadband_px").value)
        self.wrong_frames_to_trigger = int(self.get_parameter("wrong_frames_to_trigger").value)
        self.good_frames_to_release = int(self.get_parameter("good_frames_to_release").value)
        self.unknown_frames_before_recovery = int(self.get_parameter("unknown_frames_before_recovery").value)

        self.reverse_speed = float(self.get_parameter("reverse_speed").value)
        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.max_steering = float(self.get_parameter("max_steering").value)
        self.reverse_duration = float(self.get_parameter("reverse_duration").value)
        self.forward_duration = float(self.get_parameter("forward_duration").value)
        self.stop_duration = float(self.get_parameter("stop_duration").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.max_recovery_duration = float(self.get_parameter("max_recovery_duration").value)

        self.front_stop_dist = float(self.get_parameter("front_stop_dist").value)
        self.rear_stop_dist = float(self.get_parameter("rear_stop_dist").value)
        self.side_stop_dist = float(self.get_parameter("side_stop_dist").value)
        self.front_sector_deg = float(self.get_parameter("front_sector_deg").value)
        self.rear_sector_deg = float(self.get_parameter("rear_sector_deg").value)
        self.side_sector_deg = float(self.get_parameter("side_sector_deg").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout").value)
        self.use_lidar_turn_choice = bool(self.get_parameter("use_lidar_turn_choice").value)
        self.freeze_turn_direction_during_recovery = bool(
            self.get_parameter("freeze_turn_direction_during_recovery").value
        )
        self.stop_as_soon_as_good_seen = bool(
            self.get_parameter("stop_as_soon_as_good_seen").value
        )

        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.log_period = float(self.get_parameter("log_period").value)

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_cb, qos_sensor)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_sensor)
        self.cmd_pub = self.create_publisher(AckermannDrive, self.cmd_topic, 10)
        self.enable_pub = self.create_publisher(Bool, self.enable_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 5)

        self.timer = self.create_timer(1.0 / max(1.0, self.control_rate_hz), self.control_loop)

        self.direction = DirectionState()
        self.lidar = LidarState()

        self.recovery_active = False
        self.recovery_start_time = 0.0
        self.phase = "IDLE"
        self.phase_start_time = 0.0
        self.turn_dir = 1  # +1 left, -1 right

        self.wrong_count = 0
        self.good_count = 0
        self.unknown_count = 0
        self.last_log_time = 0.0
        self.enable_state = False

        self.get_logger().info(
            f"Ready: image={self.image_topic}, scan={self.scan_topic}, cmd={self.cmd_topic}, enable={self.enable_topic}, "
            f"expected_green_side={self.expected_green_side}"
        )

    # ------------------------- Image handling -------------------------

    def image_cb(self, msg: Image):
        try:
            bgr = self.ros_image_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn(f"Image conversion failed: {exc}")
            return

        direction, debug = self.analyze_image(bgr)
        self.direction = direction

        if direction.state == "WRONG":
            self.wrong_count += 1
            self.good_count = 0
            self.unknown_count = 0
        elif direction.state == "GOOD":
            self.good_count += 1
            self.wrong_count = 0
            self.unknown_count = 0
        else:
            self.unknown_count += 1
            self.good_count = 0
            # Do not reset wrong_count too aggressively: this avoids blinking detection.
            self.wrong_count = max(0, self.wrong_count - 1)

        if self.publish_debug_image:
            try:
                self.debug_pub.publish(self.bgr_to_ros_image(debug, msg.header))
            except Exception as exc:
                self.get_logger().warn(f"Debug image publish failed: {exc}")

    def ros_image_to_bgr(self, msg: Image) -> np.ndarray:
        data = np.frombuffer(msg.data, dtype=np.uint8)
        h, w = msg.height, msg.width
        enc = msg.encoding.lower()

        if enc in ("bgr8", "8uc3"):
            img = data.reshape((h, w, 3))
            return img.copy()
        if enc == "rgb8":
            img = data.reshape((h, w, 3))
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if enc == "bgra8":
            img = data.reshape((h, w, 4))
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        if enc == "rgba8":
            img = data.reshape((h, w, 4))
            return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        if enc in ("mono8", "8uc1"):
            img = data.reshape((h, w))
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        raise ValueError(f"Unsupported encoding: {msg.encoding}")

    def bgr_to_ros_image(self, bgr: np.ndarray, header) -> Image:
        out = Image()
        out.header = header
        out.height = int(bgr.shape[0])
        out.width = int(bgr.shape[1])
        out.encoding = "bgr8"
        out.is_bigendian = 0
        out.step = int(bgr.shape[1] * 3)
        out.data = bgr.tobytes()
        return out

    def analyze_image(self, bgr: np.ndarray) -> Tuple[DirectionState, np.ndarray]:
        h, w = bgr.shape[:2]
        roi_y0 = h // 2 if self.use_lower_half_only else 0
        roi = bgr[roi_y0:h, :]
        roi_h, roi_w = roi.shape[:2]
        min_area = max(self.min_color_area, int(self.min_area_ratio * roi_h * roi_w))

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # HSV thresholds. Adjust if Webots lighting/materials differ.
        red1 = cv2.inRange(hsv, np.array([0, 70, 60]), np.array([10, 255, 255]))
        red2 = cv2.inRange(hsv, np.array([170, 70, 60]), np.array([180, 255, 255]))
        red_mask = cv2.bitwise_or(red1, red2)
        green_mask = cv2.inRange(hsv, np.array([35, 55, 45]), np.array([95, 255, 255]))

        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

        red_area, red_cx = self.mask_area_centroid_x(red_mask)
        green_area, green_cx = self.mask_area_centroid_x(green_mask)

        red_side = self.cx_to_side(red_cx, roi_w) if red_area >= min_area else "unknown"
        green_side = self.cx_to_side(green_cx, roi_w) if green_area >= min_area else "unknown"

        state = "UNKNOWN"
        confidence = 0.0

        if green_area >= min_area and red_area >= min_area and green_side in ("left", "right") and red_side in ("left", "right"):
            if green_side != red_side:
                state = "GOOD" if green_side == self.expected_green_side else "WRONG"
                confidence = min(1.0, (green_area + red_area) / float(roi_h * roi_w) * 8.0)
            else:
                state = "UNKNOWN"
        elif green_area >= min_area and green_side in ("left", "right"):
            # Optional weaker decision if only green is visible.
            state = "GOOD" if green_side == self.expected_green_side else "WRONG"
            confidence = min(0.55, green_area / float(roi_h * roi_w) * 8.0)

        debug = bgr.copy()
        cv2.line(debug, (w // 2, roi_y0), (w // 2, h - 1), (255, 255, 255), 1)
        cv2.rectangle(debug, (0, roi_y0), (w - 1, h - 1), (255, 255, 255), 1)

        # Overlay masks lightly.
        color_overlay = np.zeros_like(roi)
        color_overlay[green_mask > 0] = (0, 180, 0)
        color_overlay[red_mask > 0] = (0, 0, 180)
        debug[roi_y0:h, :] = cv2.addWeighted(roi, 0.75, color_overlay, 0.25, 0)

        if green_area >= min_area and green_cx is not None:
            cv2.circle(debug, (int(green_cx), roi_y0 + roi_h // 2), 8, (0, 255, 0), -1)
        if red_area >= min_area and red_cx is not None:
            cv2.circle(debug, (int(red_cx), roi_y0 + roi_h // 2), 8, (0, 0, 255), -1)

        text = (
            f"{state}  green={green_side} area={green_area}  "
            f"red={red_side} area={red_area}  lidar F={self.lidar.front_min:.2f} R={self.lidar.rear_min:.2f}"
        )
        cv2.putText(debug, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.putText(debug, f"recovery={self.recovery_active} phase={self.phase}", (10, 58),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        return DirectionState(state, green_side, red_side, green_area, red_area, confidence), debug

    def mask_area_centroid_x(self, mask: np.ndarray) -> Tuple[int, Optional[float]]:
        m = cv2.moments(mask, binaryImage=True)
        area = int(m["m00"])
        if area <= 0:
            return 0, None
        return area, float(m["m10"] / m["m00"])

    def cx_to_side(self, cx: Optional[float], width: int) -> str:
        if cx is None:
            return "unknown"
        center = width * 0.5
        if cx < center - self.side_deadband_px:
            return "left"
        if cx > center + self.side_deadband_px:
            return "right"
        return "center"

    # ------------------------- LiDAR handling -------------------------

    def scan_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges), dtype=np.float32) * msg.angle_increment

        valid = np.isfinite(ranges)
        valid &= ranges > max(0.01, msg.range_min)
        valid &= ranges < msg.range_max
        ranges = np.where(valid, ranges, np.inf)

        def sector_min(center_deg: float, width_deg: float) -> float:
            center = math.radians(center_deg)
            half = math.radians(width_deg) * 0.5
            # Wrapped angular difference in [-pi, pi]
            diff = np.arctan2(np.sin(angles - center), np.cos(angles - center))
            vals = ranges[np.abs(diff) <= half]
            if vals.size == 0:
                return float("inf")
            v = float(np.min(vals))
            return v if math.isfinite(v) else float("inf")

        front = sector_min(0.0, self.front_sector_deg)
        rear = sector_min(180.0, self.rear_sector_deg)
        left = sector_min(90.0, self.side_sector_deg)
        right = sector_min(-90.0, self.side_sector_deg)
        left_front = sector_min(45.0, 55.0)
        right_front = sector_min(-45.0, 55.0)
        left_rear = sector_min(135.0, 55.0)
        right_rear = sector_min(-135.0, 55.0)

        # Best forward turning side = side with more front diagonal clearance.
        best_turn = 1 if left_front >= right_front else -1

        self.lidar = LidarState(
            front_min=front,
            rear_min=rear,
            left_min=left,
            right_min=right,
            left_front_min=left_front,
            right_front_min=right_front,
            left_rear_min=left_rear,
            right_rear_min=right_rear,
            best_turn=best_turn,
            stamp=time.monotonic(),
        )

    def lidar_recent(self) -> bool:
        return (time.monotonic() - self.lidar.stamp) <= self.scan_timeout

    # ------------------------- Recovery control -------------------------

    def control_loop(self):
        now = time.monotonic()

        if not self.recovery_active:
            if self.wrong_count >= self.wrong_frames_to_trigger:
                self.start_recovery(now, reason="WRONG confirmed by camera")
            elif self.unknown_count >= self.unknown_frames_before_recovery and self.wrong_count > 0:
                self.start_recovery(now, reason="camera uncertain after wrong detection")
            else:
                self.publish_enable(False)
                self.publish_cmd(0.0, 0.0)
                self.periodic_log(now)
                return

        # Active recovery:
        # If the camera already sees GOOD, stop moving immediately.
        # We keep enable=True until GOOD is stable, so Nav2 cannot take over too early.
        if self.stop_as_soon_as_good_seen and self.direction.state == "GOOD":
            self.publish_cmd(0.0, 0.0)

            if self.good_count >= self.good_frames_to_release:
                self.stop_recovery("GOOD confirmed by camera")
            return

        if self.good_count >= self.good_frames_to_release:
            self.stop_recovery("GOOD confirmed by camera")
            return

        if self.max_recovery_duration > 0.0 and (now - self.recovery_start_time) > self.max_recovery_duration:
            self.stop_recovery("max_recovery_duration reached")
            return

        self.run_recovery_phase(now)
        self.periodic_log(now)

    def start_recovery(self, now: float, reason: str):
        self.recovery_active = True
        self.publish_enable(True)
        self.recovery_start_time = now
        self.phase = "STOP_BEFORE_REVERSE"
        self.phase_start_time = now
        self.good_count = 0

        if self.use_lidar_turn_choice and self.lidar_recent():
            self.turn_dir = self.lidar.best_turn
        else:
            # Fallback: if green is currently on the wrong side, choose a deterministic turn.
            self.turn_dir = 1 if self.expected_green_side == "left" else -1

        self.get_logger().warn(
            f"Recovery started: {reason}, turn_dir={'left' if self.turn_dir > 0 else 'right'}, "
            f"state={self.direction.state}, green={self.direction.green_side}, red={self.direction.red_side}"
        )

    def stop_recovery(self, reason: str):
        self.get_logger().info(f"Recovery stopped: {reason}")
        self.recovery_active = False
        self.phase = "IDLE"
        self.phase_start_time = time.monotonic()
        self.wrong_count = 0
        self.good_count = 0
        self.unknown_count = 0
        self.publish_cmd(0.0, 0.0)
        self.publish_enable(False)

    def run_recovery_phase(self, now: float):
        elapsed = now - self.phase_start_time

        if self.phase == "STOP_BEFORE_REVERSE":
            self.publish_cmd(0.0, 0.0)
            if elapsed >= self.stop_duration:
                self.set_phase("REVERSE_TURN", now)
            return

        if self.phase == "REVERSE_TURN":
            # When reversing an Ackermann car, steering sign effect is inverted in world motion.
            # Use opposite steering from the next forward phase to create a 3-point turn.
            steering = -self.turn_dir * self.max_steering
            speed = self.reverse_speed

            if self.lidar_recent() and self.lidar.rear_min < self.rear_stop_dist:
                self.get_logger().warn(f"Rear too close ({self.lidar.rear_min:.2f} m), switching to forward")
                self.set_phase("STOP_BEFORE_FORWARD", now)
                self.publish_cmd(0.0, 0.0)
                return

            self.publish_cmd(speed, steering)
            if elapsed >= self.reverse_duration:
                self.set_phase("STOP_BEFORE_FORWARD", now)
            return

        if self.phase == "STOP_BEFORE_FORWARD":
            self.publish_cmd(0.0, 0.0)

            # Important:
            # Do not recompute turn_dir here by default.
            # If LiDAR flips the chosen side between reverse and forward, the forward phase can undo
            # the rotation made during reverse and send the car back toward the wrong direction.
            if elapsed >= self.stop_duration:
                if not self.freeze_turn_direction_during_recovery:
                    if self.use_lidar_turn_choice and self.lidar_recent():
                        self.turn_dir = self.lidar.best_turn
                self.set_phase("FORWARD_TURN", now)
            return

        if self.phase == "FORWARD_TURN":
            steering = self.turn_dir * self.max_steering
            speed = self.forward_speed

            if self.lidar_recent():
                if self.lidar.front_min < self.front_stop_dist:
                    self.get_logger().warn(f"Front too close ({self.lidar.front_min:.2f} m), switching to reverse")
                    self.set_phase("STOP_BEFORE_REVERSE", now)
                    self.publish_cmd(0.0, 0.0)
                    return
                # Do not flip steering direction during an active turn by default.
                # Flipping here can undo the 3-point turn.
                # If you disable freeze_turn_direction_during_recovery, the old adaptive behavior is restored.
                if not self.freeze_turn_direction_during_recovery:
                    if self.turn_dir > 0 and self.lidar.left_min < self.side_stop_dist:
                        self.turn_dir = -1
                    elif self.turn_dir < 0 and self.lidar.right_min < self.side_stop_dist:
                        self.turn_dir = 1

            self.publish_cmd(speed, steering)
            if elapsed >= self.forward_duration:
                self.set_phase("STOP_BEFORE_REVERSE", now)
            return

        # Unknown phase fallback.
        self.set_phase("STOP_BEFORE_REVERSE", now)
        self.publish_cmd(0.0, 0.0)

    def set_phase(self, phase: str, now: float):
        self.phase = phase
        self.phase_start_time = now

    def publish_enable(self, enabled: bool):
        enabled = bool(enabled)
        if self.enable_state == enabled:
            return
        self.enable_state = enabled
        msg = Bool()
        msg.data = enabled
        self.enable_pub.publish(msg)

    def publish_cmd(self, speed: float, steering: float):
        msg = AckermannDrive()
        msg.speed = float(speed)
        msg.steering_angle = float(max(-self.max_steering, min(self.max_steering, steering)))
        self.cmd_pub.publish(msg)

    def periodic_log(self, now: float):
        if now - self.last_log_time < self.log_period:
            return
        self.last_log_time = now
        self.get_logger().info(
            f"camera={self.direction.state} green={self.direction.green_side} red={self.direction.red_side} "
            f"counts W/G/U={self.wrong_count}/{self.good_count}/{self.unknown_count} "
            f"recovery={self.recovery_active} phase={self.phase} "
            f"lidar F/R/L/R={self.lidar.front_min:.2f}/{self.lidar.rear_min:.2f}/"
            f"{self.lidar.left_min:.2f}/{self.lidar.right_min:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ColorDirectionLidarRecovery()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.publish_enable(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
