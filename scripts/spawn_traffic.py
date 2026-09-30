#!/usr/bin/env python3
"""往场景里撒行人和车辆，让检测/避障有东西可测。

行人用 CARLA 的 controller.ai.walker，会真的在路网上走来走去；
车辆开 autopilot，会跟着车流跑。

重要：**必须在起 carla_bridge 之前跑**。
桥会把世界切成同步模式，之后第二个客户端做 spawn 会超时。

用法:
    bash setup/lab.sh start carla headless
    python scripts/spawn_traffic.py --walkers 40 --vehicles 25
    bash setup/lab.sh start bridge
    ...
    python scripts/spawn_traffic.py --clear     # 清场
"""

from __future__ import annotations

import argparse
import random
import sys
import time

import carla


def connect(timeout: float = 30.0) -> carla.Client:
    client = carla.Client("localhost", 2000)
    client.set_timeout(timeout)
    for _ in range(10):
        try:
            client.get_world()
            return client
        except RuntimeError:
            time.sleep(1)
    return client


def clear(world) -> None:
    n_w = 0
    for a in world.get_actors():
        if a.attributes.get("role_name") in ("lab_walker", "lab_walker_ctrl", "lab_traffic"):
            a.destroy()
            n_w += 1
    print(f"清掉 {n_w} 个交通参与者")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--walkers", type=int, default=40)
    ap.add_argument("--vehicles", type=int, default=25)
    ap.add_argument("--clear", action="store_true")
    args = ap.parse_args()

    client = connect()
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    if args.clear:
        clear(world)
        return 0

    # ---------------- 行人 ----------------
    walker_bps = bp_lib.filter("walker.pedestrian.*")
    ctrl_bp = bp_lib.find("controller.ai.walker")
    ctrl_bp.set_attribute("role_name", "lab_walker_ctrl")
    spawned_walkers = []
    ctrl_ids = []
    for _ in range(args.walkers):
        loc = world.get_random_location_from_navigation()
        if loc is None:
            continue
        bp = random.choice(walker_bps)
        bp.set_attribute("role_name", "lab_walker")
        if bp.has_attribute("is_invincible"):
            bp.set_attribute("is_invincible", "true")
        try:
            w = world.spawn_actor(bp, carla.Transform(loc))
        except RuntimeError:
            continue
        spawned_walkers.append(w)
        try:
            c = world.spawn_actor(ctrl_bp, carla.Transform(), attach_to=w)
            ctrl_ids.append(c)
        except RuntimeError:
            pass

    # 世界先跑几帧，控制器才有东西可跟
    time.sleep(1.0)
    for c in ctrl_ids:
        c.start()
        c.go_to_location(world.get_random_location_from_navigation())
        if hasattr(c, "set_max_speed"):
            c.set_max_speed(1.4)          # 行人速度 m/s

    # ---------------- 车辆 ----------------
    tm = client.get_trafficmanager(8000)
    tm.set_global_distance_to_leading_vehicle(2.5)
    veh_bps = [b for b in bp_lib.filter("vehicle.*")
               if int(b.get_attribute("number_of_wheels").as_int()) == 4]
    spawn_points = world.get_map().get_spawn_points()
    random.shuffle(spawn_points)
    spawned_v = 0
    for sp in spawn_points:
        if spawned_v >= args.vehicles:
            break
        bp = random.choice(veh_bps)
        bp.set_attribute("role_name", "lab_traffic")
        try:
            v = world.spawn_actor(bp, sp)
        except RuntimeError:
            continue                      # 位置被占就跳过
        v.set_autopilot(True, 8000)
        tm.vehicle_percentage_speed_difference(v, random.uniform(-10, 30))
        spawned_v += 1

    print(f"行人 {len(spawned_walkers)} 个（带 AI 控制器 {len(ctrl_ids)} 个），"
          f"车辆 {spawned_v} 辆（autopilot）")
    print("注意：这些必须在本进程退出前完成 spawn，之后才可以起 carla_bridge")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

