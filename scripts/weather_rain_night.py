#!/usr/bin/env python3
"""M1: 把地图切成 Town10HD + 夜晚 + 大雨。

用法:
    python scripts/weather_rain_night.py            # 只设天气
    python scripts/weather_rain_night.py --town     # 顺便加载 Town10HD_Opt

参考原帖效果：湿滑路面、夜间、大雨、路灯光晕。
"""

import argparse
import math
import time

import carla


TOWN = "Town10HD_Opt"


def build_night_rain() -> carla.WeatherParameters:
    """雨夜天气参数。cloudiness/precipitation/deposits 拉满，太阳压到地平线下。"""
    weather = carla.WeatherParameters()
    weather.cloudiness = 95.0
    weather.precipitation = 90.0
    weather.precipitation_deposits = 90.0     # 路面积水——这是"湿滑路面"的关键
    weather.wind_intensity = 45.0
    weather.fog_density = 30.0
    weather.fog_distance = 80.0
    weather.wetness = 95.0
    weather.scattering_intensity = 2.0
    weather.mie_scattering_scale = 0.5
    weather.rayleigh_scattering_scale = 0.3

    # 夜色：太阳高度角压到负值（日落之后），方位随意
    weather.sun_altitude_angle = -12.0
    weather.sun_azimuth_angle = 200.0
    return weather


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--town", action="store_true", help="顺便加载 Town10HD_Opt")
    parser.add_argument("--hold", type=float, default=5.0,
                        help="保持多少秒后退出（0 = 常驻）")
    args = parser.parse_args()

    client = carla.Client("localhost", 2000)
    client.set_timeout(60.0)

    if args.town:
        print(f"加载地图 {TOWN} ...")
        world = client.load_world(TOWN)
    else:
        world = client.get_world()
        print(f"沿用当前地图 {world.get_map().name}")

    world.set_weather(build_night_rain())
    # 让雨滴粒子 / 灯光慢慢起来
    world.tick()
    print("天气已设为：夜晚 + 大雨 + 积水路面")
    print(f"  sun_altitude_angle = {world.get_weather().sun_altitude_angle}")

    if args.hold <= 0:
        print("常驻模式，Ctrl+C 退出")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
    else:
        time.sleep(args.hold)
    print("完成")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

