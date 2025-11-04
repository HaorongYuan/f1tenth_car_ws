"""Wall following controller tuned for the physical F1TENTH race car.

This node implements the geometry based wall following algorithm that is
described in the University of Virginia F1TENTH assignments, but augments the
controller with the additional guard rails that are required to safely operate
on the real vehicle:

*   Range measurements are median filtered and validated so that sporadic
    outliers from the Hokuyo/Urg LiDAR do not result in aggressive steering
    commands.
*   The controller exposes all key constants (look-ahead distance, gains,
    target distance, etc.) through ROS parameters so that they can be tuned on
    the car without recompilation.
*   Speed is automatically reduced when the steering angle is large or an
    obstacle is detected in front of the car, protecting the vehicle from
    over-speeding in tight turns and from collisions.
*   Steering commands are low-pass filtered and bounded to comply with the
    servo limits of the physical Ackermann steering geometry.

The node listens to the ``/scan`` topic and publishes drive commands to the
``/drive`` topic which is consumed by the vehicle's drive stack.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


@dataclass
class WallFollowingParameters:
    """Container with configurable parameters for the wall following node."""

    theta_rad: float
    target_distance: float
    lookahead_distance: float
    kp: float
    kd: float
    max_steering_angle: float
    steering_smoothing: float
    max_speed: float
    min_speed: float
    steering_speed_slope: float
    slowdown_distance: float
    stop_distance: float
    range_window: int
    range_max_fallback: float


class WallFollowingNode(Node):
    """ROS2 node that drives the car parallel to the right hand wall."""

    def __init__(self) -> None:
        super().__init__("wall_following")

        # Declare parameters with reasonable defaults for a real vehicle.
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("command_topic", "/drive")
        self.declare_parameter("theta_deg", 60.0)
        self.declare_parameter("target_distance", 1.0)
        self.declare_parameter("lookahead_distance", 3.0)
        self.declare_parameter("kp", 0.5)
        self.declare_parameter("kd", 0.05)
        self.declare_parameter("max_steering_angle", 0.34)
        self.declare_parameter("steering_smoothing", 0.3)
        self.declare_parameter("max_speed", 2.5)
        self.declare_parameter("min_speed", 0.5)
        self.declare_parameter("steering_speed_slope", 4.0)
        self.declare_parameter("slowdown_distance", 2.0)
        self.declare_parameter("stop_distance", 0.7)
        self.declare_parameter("range_window", 2)
        self.declare_parameter("range_max_fallback", 10.0)

        self._params = self._load_parameters()

        self.get_logger().info(
            "Wall follower ready: target_distance=%.2f m, lookahead=%.2f m, max_speed=%.2f m/s",
            self._params.target_distance,
            self._params.lookahead_distance,
            self._params.max_speed,
        )

        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        command_topic = (
            self.get_parameter("command_topic").get_parameter_value().string_value
        )

        self._drive_pub = self.create_publisher(
            AckermannDriveStamped, command_topic, qos_profile=10
        )
        self._scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._scan_callback, qos_profile=10
        )

        self._filtered_steering: float = 0.0
        self._previous_error: float = 0.0
        self._previous_time: Optional[float] = None

    def _load_parameters(self) -> WallFollowingParameters:
        theta_rad = math.radians(
            self.get_parameter("theta_deg").get_parameter_value().double_value
        )
        return WallFollowingParameters(
            theta_rad=theta_rad,
            target_distance=self._get_double("target_distance"),
            lookahead_distance=self._get_double("lookahead_distance"),
            kp=self._get_double("kp"),
            kd=self._get_double("kd"),
            max_steering_angle=self._get_double("max_steering_angle"),
            steering_smoothing=self._get_double("steering_smoothing"),
            max_speed=self._get_double("max_speed"),
            min_speed=self._get_double("min_speed"),
            steering_speed_slope=self._get_double("steering_speed_slope"),
            slowdown_distance=self._get_double("slowdown_distance"),
            stop_distance=self._get_double("stop_distance"),
            range_window=int(self.get_parameter("range_window").value),
            range_max_fallback=self._get_double("range_max_fallback"),
        )

    def _get_double(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _scan_callback(self, scan: LaserScan) -> None:
        if not scan.ranges:
            self.get_logger().warning("Received empty scan message")
            return

        b = self._get_range(scan, -90.0)
        a = self._get_range(scan, math.degrees(self._params.theta_rad) - 90.0)

        denominator = a * math.sin(self._params.theta_rad)
        if math.isclose(denominator, 0.0, abs_tol=1e-6):
            self.get_logger().debug("Denominator too small, skipping control update")
            return

        alpha = math.atan2(
            a * math.cos(self._params.theta_rad) - b,
            denominator,
        )
        ab = b * math.cos(alpha)
        projected_distance = ab + self._params.lookahead_distance * math.sin(alpha)
        error = self._params.target_distance - projected_distance

        current_time = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        derivative = 0.0
        if self._previous_time is not None:
            dt = max(current_time - self._previous_time, 1e-3)
            derivative = (error - self._previous_error) / dt
        self._previous_time = current_time
        self._previous_error = error

        steering_command = self._params.kp * error + self._params.kd * derivative
        steering_command = float(
            np.clip(
                steering_command,
                -self._params.max_steering_angle,
                self._params.max_steering_angle,
            )
        )

        smoothing = np.clip(self._params.steering_smoothing, 0.0, 1.0)
        self._filtered_steering = (
            smoothing * self._filtered_steering + (1.0 - smoothing) * steering_command
        )

        front_distance = self._get_range(scan, 0.0)
        speed_command = self._compute_speed(front_distance, abs(self._filtered_steering))

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = self._filtered_steering
        drive_msg.drive.speed = speed_command
        self._drive_pub.publish(drive_msg)

    def _compute_speed(self, front_distance: float, steering_abs: float) -> float:
        # Slow down as the steering angle grows to keep within safe lateral
        # acceleration limits on the real car.
        speed_from_steering = self._params.max_speed - self._params.steering_speed_slope * steering_abs
        speed_from_steering = float(
            np.clip(speed_from_steering, self._params.min_speed, self._params.max_speed)
        )

        # Further scale the speed when an obstacle is detected ahead.
        if front_distance <= self._params.stop_distance:
            return 0.0

        if front_distance <= self._params.slowdown_distance:
            slowdown_range = max(
                self._params.slowdown_distance - self._params.stop_distance, 1e-3
            )
            scale = (front_distance - self._params.stop_distance) / slowdown_range
            scaled_speed = self._params.min_speed + (
                speed_from_steering - self._params.min_speed
            ) * scale
            return float(np.clip(scaled_speed, 0.0, self._params.max_speed))

        return speed_from_steering

    def _get_range(self, scan: LaserScan, angle_deg: float) -> float:
        angle_rad = math.radians(angle_deg)
        raw_index = (angle_rad - scan.angle_min) / scan.angle_increment
        index = int(np.clip(round(raw_index), 0, len(scan.ranges) - 1))

        window = max(self._params.range_window, 0)
        indices = np.arange(index - window, index + window + 1, dtype=int)
        indices = np.clip(indices, 0, len(scan.ranges) - 1)
        values = np.array(scan.ranges, dtype=float)[indices]

        valid = values[np.isfinite(values)]
        if valid.size == 0:
            return min(scan.range_max, self._params.range_max_fallback)

        clipped = np.clip(valid, scan.range_min, scan.range_max)
        return float(np.median(clipped))


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = WallFollowingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover - convenience for local testing
    main()
