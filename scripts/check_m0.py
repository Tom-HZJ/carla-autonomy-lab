#!/usr/bin/env python3
"""M0 验收：CARLA 服务端是否活着、客户端能否拿到世界。

用法（先启动 CARLA 服务端）:
    python scripts/check_m0.py

通过标准: 打印出 client/server 版本、地图名、spawn point 数量，且不抛异常。
"""

import sys

import carla


def main() -> int:
    print("== M0 验收 ==")
    client = carla.Client("localhost", 2000)
    client.set_timeout(15.0)

    try:
        client_version = client.get_client_version()
        server_version = client.get_server_version()
    except RuntimeError as exc:
        print(f"[FAIL] 连不上 CARLA 服务端: {exc}")
        print("       先运行: bash setup/run_carla.sh headless")
        return 1

    print(f"[OK] 客户端版本 : {client_version}")
    print(f"[OK] 服务端版本 : {server_version}")

    world = client.get_world()
    carla_map = world.get_map()
    spawn_points = carla_map.get_spawn_points()

    print(f"[OK] 当前地图   : {carla_map.name}")
    print(f"[OK] spawn 点   : {len(spawn_points)}")

    bp = world.get_blueprint_library()
    vehicles = bp.filter("vehicle.*")
    sensors = bp.filter("sensor.*")
    print(f"[OK] 蓝图库     : {len(vehicles)} 种车辆, {len(sensors)} 种传感器")

    actors = world.get_actors()
    print(f"[OK] 当前 actor : {len(actors)} 个")

    # 检查关键传感器蓝图是否齐全（后面 M2 要用）
    needed = [
        "sensor.camera.rgb",
        "sensor.lidar.ray_cast",
        "sensor.other.imu",
        "sensor.other.gnss",
    ]
    missing = [n for n in needed if len(bp.filter(n)) == 0]
    if missing:
        print(f"[WARN] 缺少传感器蓝图: {missing}")
    else:
        print("[OK] 关键传感器蓝图齐全 (rgb / lidar / imu / gnss)")

    print("== M0 通过 ==")
    return 0


if __name__ == "__main__":
    sys.exit(main())

