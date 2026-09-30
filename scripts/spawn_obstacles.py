#!/usr/bin/env python3
"""在路线前方摆几个静态障碍物，用来测避障。

用法:
    python scripts/spawn_obstacles.py                 # 默认在 3 个位置摆路障
    python scripts/spawn_obstacles.py --at 60,150,220 # 指定路线索引
    python scripts/spawn_obstacles.py --clear         # 清掉之前摆的

这些是 static.prop.* 静态道具（护栏/路锥/垃圾桶），不受物理影响，
摆下去就一直在，适合验证"看见 -> 减速/绕开"。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import carla

ROUTE = Path("/home/tom/Desktop/ROS2/ros2_ws/src/carla_autonomy/config/route_town10.json")

# 优先用这些，别的也行
PREFERRED = [
    "static.prop.streetbarrier",
    "static.prop.constructioncone",
    "static.prop.trafficcone01",
    "static.prop.trafficwarning",
    "static.prop.mailbox",
    "static.prop.dumpster",
]


def pick_prop(bp_lib):
    for name in PREFERRED:
        got = bp_lib.filter(name)
        if got:
            return got[0]
    got = bp_lib.filter("static.prop.*")
    if got:
        return got[0]
    raise RuntimeError("蓝图库里没有 static.prop.*")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--at", default="60,150,220")
    ap.add_argument("--lateral", type=float, default=0.0,
                    help="横向偏移（米，左正），0 = 摆在车道中心")
    ap.add_argument("--clear", action="store_true")
    args = ap.parse_args()

    client = carla.Client("localhost", 2000)
    client.set_timeout(30.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    existing = [a for a in world.get_actors()
                if a.attributes.get("role_name") == "lab_obstacle"]
    if args.clear:
        for a in existing:
            a.destroy()
        print(f"清掉 {len(existing)} 个障碍物")
        return 0

    pts = json.loads(ROUTE.read_text())["points"]
    n = len(pts)
    bp = pick_prop(bp_lib)
    if bp.has_attribute("role_name"):
        bp.set_attribute("role_name", "lab_obstacle")

    spawned = 0
    for raw in args.at.split(","):
        idx = int(raw) % n
        x, y = pts[idx]
        nx, ny = pts[(idx + 1) % n]
        heading = math.degrees(math.atan2(-(ny - y), nx - x))   # ROS -> CARLA 偏航
        # 横向偏移：ROS 左法向 = (-sin, cos) -> CARLA 里 y 取反
        th = math.atan2(ny - y, nx - x)
        ox, oy = -math.sin(th) * args.lateral, math.cos(th) * args.lateral
        tf = carla.Transform(
            carla.Location(x=x + ox, y=-(y + oy), z=0.4),
            carla.Rotation(yaw=heading))
        try:
            actor = world.spawn_actor(bp, tf)
            spawned += 1
            print(f"  在路线 idx={idx} ({x:.1f},{y:.1f}) 摆了 {bp.id} (actor {actor.id})")
        except RuntimeError as exc:
            print(f"  idx={idx} 摆放失败: {exc}", file=sys.stderr)

    print(f"共摆放 {spawned} 个障碍物；清除用 --clear")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

