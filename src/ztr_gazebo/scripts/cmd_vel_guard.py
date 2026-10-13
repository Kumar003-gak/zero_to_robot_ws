#!/usr/bin/env python3
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class CmdVelGuard(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_guard")
        self.max_linear = self.declare_parameter(
            "max_linear", 5.0, ParameterDescriptor(description="Max |linear.x|, m/s")).value
        self.max_angular = self.declare_parameter(
            "max_angular", 1.0, ParameterDescriptor(description="Max |angular.z|, rad/s")).value
        self.timeout_s = self.declare_parameter(
            "timeout_s", 0.7,
            ParameterDescriptor(description="Stop the car if no command for this long, s")).value
        rate_hz = self.declare_parameter(
            "rate_hz", 20.0, ParameterDescriptor(description="Output rate, Hz")).value

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Twist, "cmd_vel_safe", qos)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, qos)

        self.last_cmd = Twist()
        self.last_cmd_time = 0.0
        self.stopped = True
        self.create_timer(1.0 / rate_hz, self.on_timer)
        self.get_logger().info(
            f"limits: {self.max_linear} m/s, {self.max_angular} rad/s, timeout {self.timeout_s} s")

    def on_cmd(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_time = time.monotonic()

    def on_timer(self) -> None:
        out = Twist()
        fresh = time.monotonic() - self.last_cmd_time < self.timeout_s
        if fresh:
            out.linear.x = clamp(self.last_cmd.linear.x, self.max_linear)
            out.angular.z = clamp(self.last_cmd.angular.z, self.max_angular)
        elif not self.stopped:
            self.get_logger().warn("no command received: stopping the car")
        self.stopped = not fresh
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = CmdVelGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
