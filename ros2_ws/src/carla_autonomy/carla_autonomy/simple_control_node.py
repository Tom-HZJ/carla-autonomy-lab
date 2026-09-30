#!/usr/bin/env python3
"""最小控制节点（M3/M4 用）：给桥发一个恒定的 Twist，验证控制链路通了。

后面 M5 会被 pure_pursuit_node 取代。

用法:
    ros2 run carla_autonomy carla_control --ros-args -p speed:=6.0 -p steer:=0.0
    ros2 run carla_autonomy carla_control --ros-args -p pattern:=square
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


class SimpleControl(Node):
    def __init__(self) -> None:
        super().__init__("carla_control")
        self.declare_parameter("speed", 6.0)          # m/s
        self.declare_parameter("steer", 0.0)          # rad（前轮转角，ROS 约定左正）
        self.declare_parameter("pattern", "straight")  # straight | square | sine
        self.declare_parameter("period", 6.0)         # square 每段秒数
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("take_autopilot_off", True)

        self.topic = self.declare_parameter("control_topic", "/carla/ego/vehicle_control") \
            .value
        self.pub = self.create_publisher(Twist, self.topic, 10)
        if self.get_parameter("take_autopilot_off").value:
            self.auto_pub = self.create_publisher(Bool, "/carla/ego/set_autopilot", 10)
            self.create_timer(0.5, self._once)  # 只发一次
            self._sent = False
        self.t0 = self.get_clock().now()
        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f"控制节点启动，发布到 {self.topic}")

    def _once(self) -> None:
        if not self._sent:
            self.auto_pub.publish(Bool(data=False))
            self._sent = True

    def tick(self) -> None:
        t = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        speed = float(self.get_parameter("speed").value)
        steer = float(self.get_parameter("steer").value)
        pattern = self.get_parameter("pattern").value

        if pattern == "square":
            period = float(self.get_parameter("period").value)
            phase = int(t / period) % 4
            steer = 0.45 if phase == 0 else 0.0
        elif pattern == "sine":
            steer = 0.3 * math.sin(2 * math.pi * t / 8.0)

        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = steer
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = SimpleControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
