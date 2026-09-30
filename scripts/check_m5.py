#!/usr/bin/env python3
"""M5 验收：纯跟踪 + 速度 PID 能不能让车贴着车道中心线跑。

前提（另开终端）:
    bash setup/lab.sh start carla headless
    bash setup/lab.sh start bridge
    bash setup/lab.sh start pursuit

本脚本只做测量：
    - 累计行驶距离
    - 横向误差（cross track error）的均值 / 95 分位 / 最大值

判定：位移 > 80 m，横向误差 95 分位 < 0.5 m
"""

from __future__ import annotations

import argparse
import math
import statistics
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32


class M5Check(Node):
    def __init__(self) -> None:
        super().__init__("check_m5")
        self.xte: list = []
        self.speeds: list = []
        self.dist = 0.0
        self._last = None
        self.create_subscription(Odometry, "/carla/ego/odometry", self.on_odom, 10)
        self.create_subscription(Float32, "/carla/ego/cross_track_error", self.on_xte, 10)

    def on_xte(self, msg: Float32) -> None:
        self.xte.append(float(msg.data))

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        if self._last is not None:
            self.dist += math.dist((p.x, p.y), self._last)
        self._last = (p.x, p.y)
        v = msg.twist.twist
        self.speeds.append(math.hypot(v.linear.x, v.linear.y))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=60.0)
    args = ap.parse_args()

    rclpy.init()
    node = M5Check()
    print(f"M5 测量 {args.duration:.0f}s ...")
    t0 = time.time()
    while time.time() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.02)
    dt = time.time() - t0

    xte = sorted(node.xte)
    print("\n== 结果 ==")
    print(f"  行驶距离 : {node.dist:8.1f} m  ({node.dist / dt:.1f} m/s 平均)")
    if node.speeds:
        print(f"  车速     : 均值 {statistics.mean(node.speeds):.1f} m/s, "
              f"最大 {max(node.speeds):.1f} m/s")
    if xte:
        p95 = xte[int(len(xte) * 0.95) - 1]
        print(f"  横向误差 : 均值 {statistics.mean(xte):.3f} m, "
              f"95% {p95:.3f} m, 最大 {xte[-1]:.3f} m  ({len(xte)} 个样本)")
    else:
        print("  横向误差 : 没收到（pure_pursuit 没在跑？）")

    ok = True
    if node.dist < 80.0:
        print(f"[FAIL] 只跑了 {node.dist:.1f} m，太少")
        ok = False
    if not xte:
        print("[FAIL] 没收到横向误差")
        ok = False
    elif xte[int(len(xte) * 0.95) - 1] > 0.5:
        print("[FAIL] 横向误差 95 分位 > 0.5 m")
        ok = False

    print("== M5 通过 ==" if ok else "== M5 未通过 ==")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

