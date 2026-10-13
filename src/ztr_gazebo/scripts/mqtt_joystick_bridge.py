#!/usr/bin/env python3
import json
import threading
import time

import paho.mqtt.client as mqtt
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def make_client(client_id: str) -> mqtt.Client:
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=client_id)
    except AttributeError:
        return mqtt.Client(client_id=client_id)


class MqttJoystickBridge(Node):
    def __init__(self) -> None:
        super().__init__("mqtt_joystick_bridge")
        self.broker = self.declare_parameter(
            "broker", "broker.hivemq.com", ParameterDescriptor(description="MQTT broker host")).value
        self.port = self.declare_parameter(
            "port", 1883, ParameterDescriptor(description="MQTT broker port")).value
        self.prefix = self.declare_parameter(
            "topic_prefix", "zero-to-robot",
            ParameterDescriptor(description="MQTT topic prefix shared with the ESP32 code")).value
        teams = list(self.declare_parameter(
            "teams", [1], ParameterDescriptor(description="Team numbers to bridge")).value)
        self.max_speed = self.declare_parameter(
            "max_speed", 5.0, ParameterDescriptor(description="Speed at full joystick, m/s")).value
        self.max_turn = self.declare_parameter(
            "max_turn", 1.0, ParameterDescriptor(description="Turn rate at full joystick, rad/s")).value
        self.dead_zone = self.declare_parameter(
            "dead_zone", 10, ParameterDescriptor(description="Joystick values below this count as 0")).value
        repeat_hz = self.declare_parameter(
            "repeat_hz", 10.0,
            ParameterDescriptor(description="Rate at which the last joystick command is republished, Hz")).value
        self.hold_s = self.declare_parameter(
            "hold_s", 3.0,
            ParameterDescriptor(description="Stop the car after this long without an MQTT message, s")).value

        self.pubs = {f"{self.prefix}/team{t}/joy": self.create_publisher(Twist, f"/team{t}/cmd_vel", 10)
                     for t in teams}
        self.seen: set[str] = set()
        self.finished: set[str] = set()
        self.last: dict[str, tuple[Twist, float]] = {}
        self.lock = threading.Lock()
        self.create_timer(1.0 / repeat_hz, self.repeat)
        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/race/finished", self.on_finished, latched)

        self.client = make_client(f"{self.prefix}-ros-bridge-{id(self) % 10000}")
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self.client.reconnect_delay_set(min_delay=1, max_delay=10)
        self.client.connect_async(self.broker, self.port, keepalive=30)
        self.client.loop_start()
        self.get_logger().info(f"connecting to {self.broker}:{self.port}, topics: {', '.join(self.pubs)}")

    def on_connect(self, client, userdata, flags, rc) -> None:
        if rc != 0:
            self.get_logger().error(f"MQTT connection refused (code {rc})")
            return
        for topic in self.pubs:
            client.subscribe(topic, qos=0)
        self.get_logger().info("MQTT connected, waiting for joysticks")

    def on_disconnect(self, client, userdata, rc) -> None:
        if rc != 0:
            self.get_logger().warn("MQTT connection lost, reconnecting...")

    def on_finished(self, msg: String) -> None:
        topic = f"/{msg.data}/cmd_vel"
        for pub in self.pubs.values():
            if pub.topic_name == topic and topic not in self.finished:
                self.finished.add(topic)
                pub.publish(Twist())
                self.get_logger().info(f"{msg.data} finished: joystick off")

    def scale(self, value: float) -> float:
        value = max(-100.0, min(100.0, float(value)))
        return 0.0 if abs(value) < self.dead_zone else value / 100.0

    def on_message(self, client, userdata, message) -> None:
        pub = self.pubs.get(message.topic)
        if pub is None or pub.topic_name in self.finished:
            return
        try:
            data = json.loads(message.payload.decode())
            x, y = data["x"], data["y"]
        except (ValueError, KeyError, TypeError, UnicodeDecodeError):
            self.get_logger().warn(f"bad message on {message.topic}: {message.payload[:60]!r}")
            return
        if message.topic not in self.seen:
            self.seen.add(message.topic)
            self.get_logger().info(f"joystick connected: {message.topic}")
        twist = Twist()
        twist.linear.x = self.scale(y) * self.max_speed
        twist.angular.z = -self.scale(x) * self.max_turn
        with self.lock:
            self.last[message.topic] = (twist, time.monotonic())
        pub.publish(twist)

    def repeat(self) -> None:
        now = time.monotonic()
        with self.lock:
            items = list(self.last.items())
        for topic, (twist, stamp) in items:
            pub = self.pubs[topic]
            if pub.topic_name in self.finished:
                continue
            if now - stamp < self.hold_s:
                pub.publish(twist)
                continue
            with self.lock:
                if self.last.get(topic, (None, None))[1] == stamp:
                    del self.last[topic]
            pub.publish(Twist())
            self.get_logger().warn(f"no joystick message for {self.hold_s} s on {topic}: stopping the car")

    def destroy_node(self) -> bool:
        self.client.loop_stop()
        self.client.disconnect()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MqttJoystickBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
