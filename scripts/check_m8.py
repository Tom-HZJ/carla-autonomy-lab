#!/usr/bin/env python3
"""M8 验收 + 录像：车能不能自己绕过挡在车道中间的路障。

做的事（一条命令跑完）:
  1. 连 CARLA，在车前方 ~35m 处**现摆**一个路障（不用重启仿真）
  2. 边跑边订阅，记录：
       - 车速、障碍物距离、重规划次数
       - 沿途相机帧（可选 --video，最后用 ffmpeg 编成 mp4）
  3. 判定：车越过了路障（纵向超过它）且没有长时间卡死

前提（另开终端）:
    bash setup/lab.sh start carla headless
    bash setup/lab.sh start bridge
    bash setup/lab.sh start perception
    bash setup/lab.sh start st
    bash setup/lab.sh start replan
    bash setup/lab.sh start pursuit -p avoid_enable:=false

用法:
    python scripts/check_m8.py --duration 80 --video demo_m8_rain.mp4
"""

from __future__ import annotations

import argparse
import math
import shutil
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Int32

OUT = Path("/home/tom/Desktop/ROS2/datasets/video")


class M8Check(Node):
    def __init__(self) -> None:
        super().__init__("check_m8")
        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1,
                       durability=DurabilityPolicy.VOLATILE)
        self.xy = None
        self.yaw = 0.0
        self.speed = 0.0
        self.obs = None
        self.replans = 0
        self.frames = 0
        self.create_subscription(Odometry, "/carla/ego/odometry", self.on_odom, 10)
        self.create_subscription(Float32MultiArray, "/carla/ego/nearest_obstacle",
                                 self.on_obs, 10)
        self.create_subscription(Int32, "/carla/ego/replan_count", self.on_replan, 10)
        self.img_q = q
        self.grab = False
        self.saved = 0
        self.frame_dir = OUT / "_frames"
        self.create_subscription(Image, "/carla/ego/camera/image_raw", self.on_img, q)

    def on_odom(self, m: Odometry) -> None:
        p = m.pose.pose.position
        self.xy = (p.x, p.y)
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        v = m.twist.twist
        self.speed = math.hypot(v.linear.x, v.linear.y)

    def on_obs(self, m: Float32MultiArray) -> None:
        self.obs = tuple(m.data) if len(m.data) >= 5 else None

    def on_replan(self, m: Int32) -> None:
        self.replans = int(m.data)

    def on_img(self, m: Image) -> None:
        if not self.grab:
            return
        self.frames += 1
        if self.frames % 3:            # 20Hz 下每 3 帧存一张 ≈ 6.7fps
            return
        # 文件名必须**连续**：ffmpeg 的 f%06d 是按连号找序列的。
        # 之前直接用 frames 当序号（3、6、9…）是断号的，ffmpeg 只读到第 1 张，
        # 编出来的视频只有 1 帧。
        self.saved += 1
        try:
            arr = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
            bgr = arr[:, :, :3]
            from PIL import Image as PILImage
            PILImage.fromarray(bgr[:, :, ::-1]).save(
                self.frame_dir / f"f{self.saved:06d}.png")
        except Exception:  # noqa: BLE001
            pass


def place_barrier(client, ahead: float = 35.0):
    """在车正前方 ahead 米摆一个路障，返回它的位置。"""
    import carla
    # 新连上来的客户端第一次 get_actors() 常常返回空，必须重试
    ego, world = None, None
    for _ in range(15):
        try:
            world = client.get_world()
            for a in world.get_actors().filter("vehicle.*"):
                if a.attributes.get("role_name") == "ego_vehicle":
                    ego = a
                    break
        except Exception:  # noqa: BLE001
            pass
        if ego is not None:
            break
        time.sleep(1.0)
    if ego is None:
        raise RuntimeError("找不到 ego_vehicle")
    t = ego.get_transform()
    # 沿"路线"往前摆，而不是沿车头方向 —— 车在弯道上时，
    # 沿车头方向摆出去的点会落在路外，车根本遇不到它。
    import json
    route_file = "/home/tom/Desktop/ROS2/ros2_ws/src/carla_autonomy/config/route_town10.json"
    pts = json.loads(Path(route_file).read_text())["points"]
    n = len(pts)
    ros_xy = (t.location.x, -t.location.y)
    i0 = min(range(n), key=lambda i: math.dist(ros_xy, pts[i]))
    # ahead 米 ≈ ahead 个点（路线是按 1m 重采样的）
    tgt = pts[(i0 + int(ahead)) % n]
    world_xy = (tgt[0], -tgt[1])          # ROS -> CARLA（y 取反）
    bp = world.get_blueprint_library().filter("static.prop.streetbarrier")[0]
    if bp.has_attribute("role_name"):
        bp.set_attribute("role_name", "lab_obstacle")
    loc = carla.Location(world_xy[0], world_xy[1], 0.4)
    actor = world.spawn_actor(bp, carla.Transform(loc, carla.Rotation(yaw=t.rotation.yaw)))
    return actor, loc


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=80.0)
    ap.add_argument("--ahead", type=float, default=35.0)
    ap.add_argument("--video", default="")
    ap.add_argument("--no-barrier", action="store_true")
    args = ap.parse_args()

    import carla
    client = carla.Client("localhost", 2000)
    client.set_timeout(30.0)

    barrier, bloc = (None, None)
    if not args.no_barrier:
        # 先清掉上一轮留下的路障：否则会变成"路障还在，但不是这次摆的"，
        # 判定和实际跑的就不是同一件事。
        try:
            world0 = client.get_world()
            old = [a for a in world0.get_actors()
                   if a.attributes.get("role_name") == "lab_obstacle"]
            for a in old:
                a.destroy()
            if old:
                print(f"清掉上一轮 {len(old)} 个旧路障")
        except Exception:  # noqa: BLE001
            pass
        try:
            barrier, bloc = place_barrier(client, args.ahead)
            print(f"已在前方 {args.ahead:.0f}m 摆路障 loc=({bloc.x:.1f},{bloc.y:.1f})")
        except Exception as exc:  # noqa: BLE001
            # 关键：摆不上就**直接退出**。
            # 之前这里是"（继续跑）"，结果路障没摆上照样跑完，
            # 拿一堆跟路障无关的数据去判否 —— 假阴性就是这么来的。
            print(f"[FAIL] 摆路障失败，本次验收作废: {exc}")
            return 2

    OUT.mkdir(parents=True, exist_ok=True)
    if args.video:
        if OUT.joinpath("_frames").exists():
            shutil.rmtree(OUT / "_frames")
        (OUT / "_frames").mkdir(parents=True)

    rclpy.init()
    node = M8Check()
    node.frame_dir.mkdir(parents=True, exist_ok=True)
    node.grab = bool(args.video)

    # 等到数据流起来
    t0 = time.time()
    while node.xy is None and time.time() - t0 < 20:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.xy is None:
        print("收不到 odometry，检查桥起没起")
        return 1

    start = node.xy
    passed = False
    closest_ever = 1e9
    rising = 0
    stuck_from = None
    max_stuck = 0.0
    min_obs_d = 1e9
    print(f"开始跑 {args.duration:.0f}s ...")
    t0 = time.time()
    while time.time() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.02)
        if node.obs is not None:
            min_obs_d = min(min_obs_d, node.obs[3])
        # 判定"越过"：不能看障碍物纵向坐标 —— 检测器的 ROI 只往前看
        # (x>3m)，车一旦越过它就不在话题里了，xt 永远变不成负数。
        # 改成盯着"到路障的世界距离"：先逼近到很近，再一路变远，
        # 而且车还在动 —— 那就是绕过去了。
        if barrier is not None and node.xy is not None:
            d = math.dist(node.xy, (bloc.x, -bloc.y))   # bloc 是 CARLA 坐标
            closest_ever = min(closest_ever, d)
            # 判据放简单点：曾经逼近到 12m 以内，现在又跑到 25m 开外。
            # 之前那版要求"连续 25 帧距离都在变大 且 车速>1"，
            # 结果车在重规划时短暂掉速就把判定打断了，报了假阴性。
            if closest_ever < 12.0 and d > 25.0:
                passed = True
        if node.speed < 0.3:
            stuck_from = stuck_from or time.time()
            max_stuck = max(max_stuck, time.time() - stuck_from)
        else:
            stuck_from = None
        node.grab = bool(args.video) and (time.time() - t0 > 2)

    total = math.dist(node.xy, start) if node.xy else 0.0
    print("\n== 结果 ==")
    print(f"  终点位移（相对起点） : {total:.1f} m")
    print(f"  重规划次数           : {node.replans}")
    print(f"  最近接近障碍物       : {min_obs_d:.1f} m")
    print(f"  最长连续停车         : {max_stuck:.1f} s")
    print(f"  越过障碍物           : {'是' if passed else '否'}")
    if args.video:
        print(f"  抓到相机帧           : {node.frames} 帧")

    node.destroy_node()
    rclpy.shutdown()

    if args.video and node.saved > 10:
        out = OUT / args.video
        cmd = ["ffmpeg", "-y", "-framerate", "7", "-i",
               str(OUT / "_frames" / "f%06d.png"),
               "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23", str(out)]
        r = subprocess.run(cmd, capture_output=True)
        probe = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0",
                                "-show_entries", "stream=nb_frames",
                                "-of", "default=nw=1:nk=1", str(out)],
                               capture_output=True)
        nfr = probe.stdout.decode().strip()
        print(f"  视频: {out}  ({'成功' if r.returncode == 0 else '失败'}) "
              f"共 {nfr} 帧 / 存了 {node.saved} 张")
        if r.returncode != 0:
            print(r.stderr.decode()[-400:])

    ok = passed and max_stuck < 25.0
    print("== M8 通过 ==" if ok else "== M8 未通过 ==")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
