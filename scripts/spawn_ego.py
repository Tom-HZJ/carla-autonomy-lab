#!/usr/bin/env python3
"""M2: 生成自车 + Camera/LiDAR/IMU/GNSS，并把传感器数据落盘。

用法:
    python scripts/spawn_ego.py                       # 默认 8 秒，存 3 路数据
    python scripts/spawn_ego.py --duration 15 --view  # 顺便把 spectator 跟住车
    python scripts/spawn_ego.py --no-save             # 只跑不存

产物:
    datasets/camera/rgb_XXXXXX.png
    datasets/lidar/lidar_XXXXXX.ply
    datasets/imu/imu.csv, datasets/gnss/gnss.csv
"""

from __future__ import annotations

import argparse
import csv
import queue
import random
import time
from pathlib import Path

import carla


PROJ = Path("/home/tom/Desktop/ROS2")


def night_rain() -> carla.WeatherParameters:
    w = carla.WeatherParameters()
    w.cloudiness = 95.0
    w.precipitation = 90.0
    w.precipitation_deposits = 90.0
    w.wind_intensity = 45.0
    w.fog_density = 30.0
    w.fog_distance = 80.0
    w.wetness = 95.0
    w.scattering_intensity = 2.0
    w.mie_scattering_scale = 0.5
    w.rayleigh_scattering_scale = 0.3
    w.sun_altitude_angle = -12.0
    w.sun_azimuth_angle = 200.0
    return w


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=8.0)
    ap.add_argument("--view", action="store_true", help="spectator 跟随自车")
    ap.add_argument("--no-save", action="store_true")
    ap.add_argument("--vehicle", default="vehicle.tesla.model3")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--lidar-channels", type=int, default=32)
    args = ap.parse_args()

    save = not args.no_save
    out = {
        "camera": PROJ / "datasets/camera",
        "lidar": PROJ / "datasets/lidar",
        "imu": PROJ / "datasets/imu",
        "gnss": PROJ / "datasets/gnss",
    }
    if save:
        for p in out.values():
            p.mkdir(parents=True, exist_ok=True)
        for old in out["camera"].glob("rgb_*.png"):
            old.unlink()
        for old in out["lidar"].glob("lidar_*.ply"):
            old.unlink()

    client = carla.Client("localhost", 2000)
    client.set_timeout(30.0)
    world = client.get_world()
    world.set_weather(night_rain())
    print(f"地图: {world.get_map().name}   天气: 雨夜")

    bp_lib = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()

    # --- 自车 ---
    vbp = bp_lib.filter(args.vehicle)[0]
    if vbp.has_attribute("role_name"):
        vbp.set_attribute("role_name", "ego_vehicle")
    ego = world.spawn_actor(vbp, random.choice(spawn_points))
    ego.set_autopilot(False)
    print(f"自车: {vbp.id}  id={ego.id}")

    # --- 摄像头 ---
    cbp = bp_lib.find("sensor.camera.rgb")
    cbp.set_attribute("image_size_x", str(args.width))
    cbp.set_attribute("image_size_y", str(args.height))
    cbp.set_attribute("fov", "90")
    cam_tf = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera = world.spawn_actor(cbp, cam_tf, attach_to=ego)

    # --- LiDAR ---
    lbp = bp_lib.find("sensor.lidar.ray_cast")
    lbp.set_attribute("channels", str(args.lidar_channels))
    lbp.set_attribute("range", "80")
    lbp.set_attribute("rotation_frequency", "20")
    lbp.set_attribute("points_per_second", "600000")
    lidar = world.spawn_actor(lbp, carla.Transform(carla.Location(x=0.0, z=2.5)), attach_to=ego)

    # --- IMU / GNSS ---
    ibp = bp_lib.find("sensor.other.imu")
    gbp = bp_lib.find("sensor.other.gnss")
    imu = world.spawn_actor(ibp, carla.Transform(carla.Location(x=0, z=1.0)), attach_to=ego)
    gnss = world.spawn_actor(gbp, carla.Transform(carla.Location(x=0, z=1.0)), attach_to=ego)

    counters = {"camera": 0, "lidar": 0}
    imu_rows: list = []
    gnss_rows: list = []

    def on_camera(image: carla.Image) -> None:
        counters["camera"] += 1
        if save and counters["camera"] % 10 == 1:      # 20fps 下每 0.5s 存一张
            image.save_to_disk(str(out["camera"] / f"rgb_{image.frame:06d}.png"))

    def on_lidar(meas: carla.LidarMeasurement) -> None:
        counters["lidar"] += 1
        if save and counters["lidar"] % 20 == 1:       # 每 1s 存一帧
            meas.save_to_disk(str(out["lidar"] / f"lidar_{meas.frame:06d}.ply"))

    def on_imu(meas) -> None:
        if save and len(imu_rows) < 2000:
            a = meas.accelerometer
            g = meas.gyroscope
            imu_rows.append((meas.frame, meas.timestamp,
                             a.x, a.y, a.z, g.x, g.y, g.z, meas.compass))

    def on_gnss(meas) -> None:
        if save and len(gnss_rows) < 2000:
            gnss_rows.append((meas.frame, meas.timestamp,
                              meas.latitude, meas.longitude, meas.altitude))

    camera.listen(on_camera)
    lidar.listen(on_lidar)
    imu.listen(on_imu)
    gnss.listen(on_gnss)

    # 让车稍微动起来，画面里才有东西
    tm = client.get_trafficmanager(8000)
    ego.set_autopilot(True, 8000)
    tm.vehicle_percentage_speed_difference(ego, 40.0)   # 限速的 60%

    print(f"采集 {args.duration:.0f} 秒 ...")
    t0 = time.time()
    try:
        while time.time() - t0 < args.duration:
            if args.view:
                tr = ego.get_transform()
                world.get_spectator().set_transform(
                    carla.Transform(tr.location + carla.Location(z=6, x=-8),
                                    carla.Rotation(pitch=-15, yaw=tr.rotation.yaw)))
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass

    print(f"摄像头帧数 {counters['camera']}, LiDAR 帧数 {counters['lidar']}")

    camera.stop()
    lidar.stop()
    imu.stop()
    gnss.stop()

    if save:
        with (out["imu"] / "imu.csv").open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["frame", "timestamp", "acc_x", "acc_y", "acc_z",
                        "gyro_x", "gyro_y", "gyro_z", "compass"])
            w.writerows(imu_rows)
        with (out["gnss"] / "gnss.csv").open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["frame", "timestamp", "lat", "lon", "alt"])
            w.writerows(gnss_rows)
        print(f"已保存: {len(imu_rows)} 条 IMU, {len(gnss_rows)} 条 GNSS")

    for actor in (camera, lidar, imu, gnss, ego):
        actor.destroy()
    print("M2 采集完成")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

