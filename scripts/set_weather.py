#!/usr/bin/env python3
"""在仿真跑着的时候热切换天气（不用重启 CARLA / 桥）。

用法:
    python scripts/set_weather.py --list
    python scripts/set_weather.py clear          # 白天晴天
    python scripts/set_weather.py rain_day       # 白天雨
    python scripts/set_weather.py dusk           # 黄昏
    python scripts/set_weather.py --cycle 6      # 每隔 6 秒轮一遍所有天气

天气会同时应用到 world 和所有已存在的相机（改完立刻能看出来）。
"""

from __future__ import annotations

import argparse
import time

import carla


def build(name: str) -> carla.WeatherParameters:
    """和 carla_bridge_node 里的预设保持一致。"""
    w = carla.WeatherParameters()
    if name in ("clear", "noon"):
        w.cloudiness, w.precipitation, w.precipitation_deposits = 10.0, 0.0, 0.0
        w.wind_intensity, w.fog_density, w.fog_distance = 5.0, 2.0, 200.0
        w.wetness, w.sun_altitude_angle, w.sun_azimuth_angle = 0.0, 60.0, 180.0
    elif name == "cloudy":
        w.cloudiness, w.precipitation, w.wind_intensity = 80.0, 0.0, 20.0
        w.fog_density, w.wetness, w.sun_altitude_angle = 10.0, 0.0, 45.0
    elif name == "rain_day":
        w.cloudiness, w.precipitation, w.precipitation_deposits = 90.0, 70.0, 70.0
        w.wind_intensity, w.fog_density, w.fog_distance = 30.0, 15.0, 120.0
        w.wetness, w.sun_altitude_angle = 80.0, 35.0
    elif name == "dusk":
        w.cloudiness, w.precipitation, w.wind_intensity = 40.0, 0.0, 10.0
        w.fog_density, w.wetness = 8.0, 0.0
        w.sun_altitude_angle, w.sun_azimuth_angle = 3.0, 90.0
    elif name == "fog":
        w.cloudiness, w.precipitation, w.precipitation_deposits = 70.0, 20.0, 30.0
        w.fog_density, w.fog_distance, w.wetness = 70.0, 35.0, 60.0
        w.sun_altitude_angle = 25.0
    elif name == "rain_night":
        w.cloudiness, w.precipitation, w.precipitation_deposits = 95.0, 90.0, 90.0
        w.wind_intensity, w.fog_density, w.fog_distance = 45.0, 30.0, 80.0
        w.wetness, w.scattering_intensity = 95.0, 2.0
        w.mie_scattering_scale, w.rayleigh_scattering_scale = 0.5, 0.3
        w.sun_altitude_angle, w.sun_azimuth_angle = -12.0, 200.0
    else:
        raise ValueError(f"未知天气 {name}")
    return w


ALL = ["clear", "cloudy", "rain_day", "dusk", "fog", "rain_night"]


def apply(world, name: str) -> None:
    world.set_weather(build(name))
    world.tick()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("name", nargs="?", default="clear")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--cycle", type=float, default=0.0, help="每隔 N 秒轮换所有天气")
    args = ap.parse_args()

    if args.list:
        print("可用天气:", ", ".join(ALL))
        return 0

    client = carla.Client("localhost", 2000)
    client.set_timeout(30.0)
    world = client.get_world()

    if args.cycle > 0:
        i = 0
        while True:
            name = ALL[i % len(ALL)]
            apply(world, name)
            print(f"[{time.strftime('%H:%M:%S')}] 切换到 {name}")
            i += 1
            time.sleep(args.cycle)
    else:
        apply(world, args.name)
        print(f"已切换到 {args.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

