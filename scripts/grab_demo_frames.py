#!/usr/bin/env python3
"""从正在跑的仿真里抓几帧车载相机图，存成 png，用来看出片效果。

用法:
    python scripts/grab_demo_frames.py --count 6 --interval 1.0
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import rclpy
from PIL import Image
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import Image as RosImage

OUT = Path("/home/tom/Desktop/ROS2/datasets/camera")


class Grabber(Node):
    def __init__(self) -> None:
        super().__init__("grab_demo_frames")
        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1,
                       durability=DurabilityPolicy.VOLATILE)
        self.latest = None
        self.count = 0
        self.create_subscription(RosImage, "/carla/ego/camera/image_raw", self.on_img, q)

    def on_img(self, msg: RosImage) -> None:
        self.latest = (msg.height, msg.width, bytes(msg.data))
        self.count += 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=6)
    ap.add_argument("--interval", type=float, default=1.5)
    ap.add_argument("--prefix", default="demo")
    args = ap.parse_args()

    OUT.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Grabber()

    saved = 0
    t_end = time.time() + args.count * args.interval + 20
    last_save = 0.0
    while saved < args.count and time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.latest and time.time() - last_save >= args.interval:
            h, w, raw = node.latest
            arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 4)
            bgr = arr[:, :, :3]                      # bgra -> bgr
            img = Image.fromarray(bgr[:, :, ::-1])   # -> rgb
            path = OUT / f"{args.prefix}_{saved:02d}.png"
            img.save(path)
            saved += 1
            last_save = time.time()
            print(f"saved {path}  ({w}x{h})")

    print(f"共保存 {saved} 帧，收到 {node.count} 帧")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if saved else 1


if __name__ == "__main__":
    raise SystemExit(main())

