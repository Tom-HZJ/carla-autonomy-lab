#!/usr/bin/env python3
"""直接读 nuScenes 的标注 json，把 6 路环视相机的标定吃透。

不依赖 nuscenes-devkit，只用标准库 + numpy，看得见每一层结构。

用法:
    python scripts/inspect_nuscenes.py                 # 打印概览 + 6 路标定
    python scripts/inspect_nuscenes.py --scene 0       # 只描述第 0 个场景
    python scripts/inspect_nuscenes.py --sample 0      # 把第 0 帧的 LiDAR 投到相机上
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path("/home/tom/Desktop/ROS2/subprojects/open-source-ad/data")
META = ROOT / "v1.0-mini"
CAMS = ["CAM_FRONT", "CAM_FRONT_LEFT", "CAM_FRONT_RIGHT",
        "CAM_BACK", "CAM_BACK_LEFT", "CAM_BACK_RIGHT"]


def load(name: str) -> list:
    return json.loads((META / f"{name}.json").read_text())


def quat_to_rot(q: list) -> np.ndarray:
    """nuScenes 四元数是 [w, x, y, z]（注意和 ROS 的 [x,y,z,w] 不一样）"""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", type=int, default=-1)
    ap.add_argument("--sample", type=int, default=-1)
    args = ap.parse_args()

    scenes = load("scene")
    samples = load("sample")
    calib = load("calibrated_sensor")
    sensors = load("sensor")
    sample_data = load("sample_data")

    print("=" * 68)
    print(f"nuScenes v1.0-mini  @  {ROOT}")
    print(f"  场景 {len(scenes)}  样本(关键帧) {len(samples)}  "
          f"传感器标定 {len(calib)}  sample_data {len(sample_data)}")
    print("=" * 68)

    # 标定 -> 通道名。
    # 注意方向：是 calibrated_sensor 里带 sensor_token 指向 sensor，
    # 不是 sensor 里带 calibrated_sensor_token（第一次写反了）。
    chan_of_sensor = {s["token"]: s["channel"] for s in sensors}
    cal_name = {c["token"]: chan_of_sensor.get(c["sensor_token"], "?")
                for c in calib}

    print("\n【6 路环视相机的内参和外参】")
    print(f"{'channel':<16} {'fx':>8} {'fy':>8} {'cx':>8} {'cy':>8}   "
          f"位置(x,y,z) m                 朝向四元数(w,x,y,z)")
    # 数据集里同一路相机有多套标定版本（120 条），每路只取第一条
    seen = set()
    for c in calib:
        ch = cal_name.get(c["token"], "?")
        if ch not in CAMS or ch in seen:
            continue
        seen.add(ch)
        k = c["camera_intrinsic"]
        fx, fy, cx, cy = k[0][0], k[1][1], k[0][2], k[1][2]
        t = c["translation"]
        q = c["rotation"]
        print(f"{ch:<16} {fx:8.1f} {fy:8.1f} {cx:8.1f} {cy:8.1f}   "
              f"({t[0]:6.3f},{t[1]:6.3f},{t[2]:6.3f})   "
              f"({q[0]:.3f},{q[1]:.3f},{q[2]:.3f},{q[3]:.3f})")

    print("\n【和我们的 CARLA 对比】")
    print("  nuScenes : 6 路 × 70° FOV，装在不同朝向，必须做 BEV 融合")
    print("  我们     : 1 路前视 90° FOV，只能看前方 —— 这就是为什么")
    print("             UniAD/VAD 这类模型搬不进来，只能学思路")

    if args.scene >= 0:
        s = scenes[args.scene]
        n = int(s["nbr_samples"])
        print(f"\n【场景 {args.scene}】{s['name']}  样本数 {n}  "
              f"时长约 {n * 0.5:.0f} 秒（关键帧 2Hz）")
        print(f"  描述: {s['description']}")

    if args.sample >= 0:
        sd_all = [d for d in sample_data if d["is_key_frame"]]
        lidar = next(d for d in sd_all if cal_name.get(d["calibrated_sensor_token"])
                     == "LIDAR_TOP")
        print(f"\n【LiDAR 投影演示】sample_data token={lidar['token'][:8]}...")
        pc_path = ROOT / "sweeps/LIDAR_TOP" / Path(lidar["filename"]).name
        if not pc_path.exists():
            pc_path = ROOT / "samples/LIDAR_TOP" / Path(lidar["filename"]).name
        if pc_path.exists():
            pts = np.fromfile(pc_path, dtype=np.float32).reshape(-1, 5)[:, :3]
            print(f"  点云 {pts.shape[0]} 点，范围 x[{pts[:,0].min():.1f},{pts[:,0].max():.1f}]"
                  f" y[{pts[:,1].min():.1f},{pts[:,1].max():.1f}]")
            # 取一个相机的内参，把点投上去，看落在图像内的比例
            cam_cal = next(c for c in calib
                           if cal_name.get(c["token"]) == "CAM_FRONT")
            k = np.array(cam_cal["camera_intrinsic"])
            # 相机外参（相对自车）的逆：把点转到相机系
            t = np.array(cam_cal["translation"])
            r = quat_to_rot(cam_cal["rotation"])
            pc_cam = (pts - t) @ r          # R^T @ (p - t) 的等价写法
            front = pc_cam[pc_cam[:, 2] > 0.5]
            if len(front):
                uv = (k @ front.T).T
                uv = uv[:, :2] / uv[:, 2:3]
                inside = ((uv[:, 0] > 0) & (uv[:, 0] < 1600) &
                          (uv[:, 1] > 0) & (uv[:, 1] < 900)).sum()
                print(f"  相机前方点 {len(front)}，投影落进 1600x900 画幅的有 {inside} 个 "
                      f"({inside / len(front) * 100:.1f}%)")
        else:
            print(f"  找不到点云文件 {pc_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
