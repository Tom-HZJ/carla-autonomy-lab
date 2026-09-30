#!/usr/bin/env python3
"""M3 验收：CARLA -> ROS 2 的桥是否真的通了，并且控制能不能开回来。

做的事：
  1. 订阅 camera / lidar / imu / gnss / odometry
  2. 同时以 20Hz 往 /carla/ego/vehicle_control 发 Twist（8 m/s，正弦转向）
  3. 跑 N 秒后统计各话题收到多少帧、车走了多远

判定：
  - 五路话题都有数据
  - odometry 速率 >= 15 Hz（同步模式设的是 20Hz）
  - 车辆位移 > 5 m（说明控制链路真的驱动了车）

注意：摄像头每帧 3.7MB，Python 订阅者本身是个瓶颈，
所以相机只要求「收到过且分辨率正确」，不卡速率。
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import Image, Imu, NavSatFix, PointCloud2


def sensor_qos() -> QoSProfile:
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST,
                      depth=1,
                      durability=DurabilityPolicy.VOLATILE)


class M3Check(Node):
    def __init__(self) -> None:
        super().__init__("check_m3")
        q = sensor_qos()
        self.n = {"camera": 0, "lidar": 0, "imu": 0, "gnss": 0, "odom": 0}
        self.cam_shape = None
        self.lidar_pts = None
        self.first_xy = None
        self.last_xy = None

        self.create_subscription(Image, "/carla/ego/camera/image_raw", self.on_cam, q)
        self.create_subscription(PointCloud2, "/carla/ego/lidar/points", self.on_lidar, q)
        self.create_subscription(Imu, "/carla/ego/imu", self.on_imu, q)
        self.create_subscription(NavSatFix, "/carla/ego/gnss", self.on_gnss, q)
        self.create_subscription(Odometry, "/carla/ego/odometry", self.on_odom, 10)

        self.pub_ctrl = self.create_publisher(Twist, "/carla/ego/vehicle_control", 10)
        self.t0 = time.time()
        self.create_timer(0.05, self.send_cmd)

    def send_cmd(self) -> None:
        t = time.time() - self.t0
        msg = Twist()
        msg.linear.x = 8.0
        msg.angular.z = 0.15 * math.sin(2 * math.pi * t / 10.0)
        self.pub_ctrl.publish(msg)

    def on_cam(self, m: Image) -> None:
        self.n["camera"] += 1
        self.cam_shape = (m.width, m.height, m.encoding)

    def on_lidar(self, m: PointCloud2) -> None:
        self.n["lidar"] += 1
        self.lidar_pts = m.width

    def on_imu(self, m: Imu) -> None:
        self.n["imu"] += 1

    def on_gnss(self, m: NavSatFix) -> None:
        self.n["gnss"] += 1

    def on_odom(self, m: Odometry) -> None:
        self.n["odom"] += 1
        xy = (m.pose.pose.position.x, m.pose.pose.position.y)
        if self.first_xy is None:
            self.first_xy = xy
        self.last_xy = xy


def main() -> int:
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=12.0)
    args = ap.parse_args()

    rclpy.init()
    node = M3Check()
    print(f"M3 验收中，采样 {args.duration:.0f}s ...")
    t0 = time.time()
    while time.time() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.02)
    dt = time.time() - t0

    n = node.n
    print("\n== 结果 ==")
    for k in ("camera", "lidar", "imu", "gnss", "odom"):
        print(f"  {k:7s}: {n[k]:5d} 帧  ({n[k] / dt:6.1f} Hz)")
    print(f"  相机画面: {node.cam_shape}")
    print(f"  点云点数: {node.lidar_pts}")

    moved = 0.0
    if node.first_xy and node.last_xy:
        moved = math.dist(node.first_xy, node.last_xy)
    print(f"  车辆位移: {moved:.2f} m")

    ok = True
    for k in ("camera", "lidar", "imu", "gnss", "odom"):
        if n[k] == 0:
            print(f"[FAIL] {k} 一帧都没收到")
            ok = False
    if n["odom"] / dt < 15.0:
        print(f"[FAIL] odometry 只有 {n['odom'] / dt:.1f} Hz，同步模式应该是 20Hz")
        ok = False
    if node.cam_shape and node.cam_shape[2] != "bgra8":
        print(f"[FAIL] 相机编码是 {node.cam_shape[2]}，期望 bgra8")
        ok = False
    if moved < 5.0:
        print(f"[FAIL] 车只动了 {moved:.2f} m，控制链路可能没通")
        ok = False

    print("== M3 通过 ==" if ok else "== M3 未通过 ==")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

