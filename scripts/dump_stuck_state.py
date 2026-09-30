#!/usr/bin/env python3
"""卡死现场取证：车停下不动时，把周围所有相关状态一次性 dump 出来。

不再靠"改一版跑一遍"猜，而是把卡死那一刻的输入原样抓下来单独分析。

抓到的东西（存 json + 打印摘要）:
  - 车的位置/朝向/速度/最近路径索引
  - /carla/ego/nearest_obstacle 的原始内容（x,y,半宽,距离,点数）
  - /carla/ego/obstacle_count（场上几个簇）
  - /carla/ego/st_speed_cmd 和 /carla/ego/st_profile（规划器输出了什么）
  - 重规划次数、最近一次重规划下发的路径长度

用法:
    python scripts/dump_stuck_state.py --hold 3.0 --out logs/stuck.json
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int32


class Dumper(Node):
    def __init__(self) -> None:
        super().__init__("dump_stuck")
        self.d = {"samples": []}
        self.create_subscription(Odometry, "/carla/ego/odometry", self.on_odom, 10)
        self.create_subscription(Float32MultiArray, "/carla/ego/nearest_obstacle",
                                 self.on_obs, 10)
        self.create_subscription(Int32, "/carla/ego/obstacle_count", self.on_cnt, 10)
        self.create_subscription(Float32, "/carla/ego/st_speed_cmd", self.on_st, 10)
        self.create_subscription(Float32MultiArray, "/carla/ego/st_profile",
                                 self.on_prof, 10)
        self.create_subscription(Int32, "/carla/ego/replan_count", self.on_repl, 10)
        self.create_subscription(RosPath, "/carla/ego/route_override",
                                 self.on_path, 10)

    def on_odom(self, m: Odometry) -> None:
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        t = m.twist.twist
        self.d.update({
            "xy": [round(p.x, 3), round(p.y, 3)],
            "yaw_deg": round(math.degrees(math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))), 2),
            "speed": round(math.hypot(t.linear.x, t.linear.y), 3),
        })

    def on_obs(self, m: Float32MultiArray) -> None:
        self.d["nearest_obstacle"] = [round(float(v), 3) for v in m.data]

    def on_cnt(self, m: Int32) -> None:
        self.d["obstacle_count"] = int(m.data)

    def on_st(self, m: Float32) -> None:
        self.d["st_speed_cmd"] = round(float(m.data), 3)

    def on_prof(self, m: Float32MultiArray) -> None:
        self.d["st_profile_head"] = [round(float(v), 2) for v in m.data[:10]]

    def on_repl(self, m: Int32) -> None:
        self.d["replan_count"] = int(m.data)

    def on_path(self, m: RosPath) -> None:
        self.d["override_path_len"] = len(m.poses)
        if m.poses:
            self.d["override_path_head"] = [
                [round(p.pose.position.x, 2), round(p.pose.position.y, 2)]
                for p in m.poses[:3]]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hold", type=float, default=3.0, help="连续不动多少秒算卡死")
    ap.add_argument("--timeout", type=float, default=150.0)
    ap.add_argument("--out", default="logs/stuck.json")
    args = ap.parse_args()

    rclpy.init()
    n = Dumper()
    t0 = time.time()
    stuck_since = None
    print(f"盯着看，连续不动 {args.hold:.1f}s 就取证 ...")
    while time.time() - t0 < args.timeout:
        rclpy.spin_once(n, timeout_sec=0.05)
        v = n.d.get("speed")
        if v is None:
            continue
        if v < 0.3:
            stuck_since = stuck_since or time.time()
            if time.time() - stuck_since >= args.hold:
                break
        else:
            stuck_since = None
    else:
        print("超时也没抓到卡死（车一直在动？）")
        n.destroy_node()
        rclpy.shutdown()
        return 1

    out = Path("/home/tom/Desktop/ROS2") / args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(n.d, ensure_ascii=False, indent=1))
    print("\n=== 卡死现场 ===")
    for k, v in n.d.items():
        print(f"  {k:20s} = {v}")
    print(f"\n已存 {out}")
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

