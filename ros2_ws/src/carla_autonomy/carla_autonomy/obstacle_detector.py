#!/usr/bin/env python3
"""M7a: 从 LiDAR 点云里把障碍物抠出来（自研轻量聚类，不依赖 scipy/sklearn）。

流程:
    点云 -> 只留车前方走廊内的点 -> 去掉地面 -> 体素栅格 -> 并查集聚类
         -> 每簇算质心/包围盒/最近距离 -> 发出去

发布:
    /carla/ego/obstacles            visualization_msgs/MarkerArray（RViz 看包围盒）
    /carla/ego/nearest_obstacle     std_msgs/Float32MultiArray
                                    布局 = [x, y, 半宽, 最近距离, 点数]
                                    坐标是车体系（x 前, y 左），没有目标时为空
    /carla/ego/obstacle_count       std_msgs/Int32

用法:
    ros2 run carla_autonomy obstacle_detector
"""

from __future__ import annotations

import math
import time
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Float32MultiArray, Int32
from visualization_msgs.msg import Marker, MarkerArray


SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1,
                        durability=DurabilityPolicy.VOLATILE)


class DisjointSet:
    """并查集，用来把相邻体素合并成一个簇。"""

    __slots__ = ("parent",)

    def __init__(self, n: int) -> None:
        self.parent = list(range(n))

    def find(self, x: int) -> int:
        p = self.parent
        while p[x] != x:
            p[x] = p[p[x]]
            x = p[x]
        return x

    def union(self, a: int, b: int) -> None:
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.parent[rb] = ra


class ObstacleDetector(Node):
    def __init__(self) -> None:
        super().__init__("obstacle_detector")

        # 自车滤除：LiDAR 装在车顶 2.5m 高，斜下方会打到自己的引擎盖。
        # Tesla Model 3 车长 4.7m，车头在中心前方 ~2.4m，所以 x<3m 一律丢掉。
        # 不滤的话，探测器会稳定报出"前方 2.4m 有个障碍物"，
        # 车就会对着自己的车头一直刹车 —— 这个坑很隐蔽。
        self.declare_parameter("roi_x_min", 3.0)
        self.declare_parameter("roi_x_max", 40.0)
        self.declare_parameter("roi_y", 7.0)
        # base_link 原点在轮子接地点，地面 ≈ z0。
        # 只留"离地 30cm 以上"的点，这样地面点被滤掉，
        # 而护栏/车/人这些真正挡路的物体（都高于 30cm）还留着。
        # 只按高度阈值是分不开地面和障碍物底部的，所以取一个够高的门限最省事。
        self.declare_parameter("ground_z", 0.30)
        self.declare_parameter("top_z", 2.5)
        self.declare_parameter("voxel", 0.35)
        self.declare_parameter("min_points", 4)
        self.declare_parameter("max_clusters", 12)
        self.declare_parameter("max_cluster_width", 4.0)
        self.declare_parameter("corridor_half_width", 2.6)
        self.declare_parameter("vis", True)

        self.roi_x_min = float(self.get_parameter("roi_x_min").value)
        self.roi_x_max = float(self.get_parameter("roi_x_max").value)
        self.roi_y = float(self.get_parameter("roi_y").value)
        self.ground_z = float(self.get_parameter("ground_z").value)
        self.top_z = float(self.get_parameter("top_z").value)
        self.voxel = float(self.get_parameter("voxel").value)
        self.min_points = int(self.get_parameter("min_points").value)
        self.max_clusters = int(self.get_parameter("max_clusters").value)
        self.max_cluster_width = float(self.get_parameter("max_cluster_width").value)
        self.corridor = float(self.get_parameter("corridor_half_width").value)
        self.vis = bool(self.get_parameter("vis").value)

        self.create_subscription(PointCloud2, "/carla/ego/lidar/points",
                                 self.on_cloud, SENSOR_QOS)
        self.pub_markers = self.create_publisher(MarkerArray, "/carla/ego/obstacles", 10)
        self.pub_nearest = self.create_publisher(
            Float32MultiArray, "/carla/ego/nearest_obstacle", 10)
        self.pub_count = self.create_publisher(Int32, "/carla/ego/obstacle_count", 10)

        self._last_log = 0.0
        self._ms = 0.0
        self.get_logger().info(
            f"障碍物检测就绪：走廊 |y|<{self.roi_y:.1f}m，"
            f"x∈[{self.roi_x_min:.1f},{self.roi_x_max:.1f}]，体素 {self.voxel:.2f}m")

    def on_cloud(self, msg: PointCloud2) -> None:
        t0 = time.perf_counter()
        pts = self._cloud_to_xyz(msg)
        if pts is None or len(pts) < 20:
            return
        clusters = self._cluster(pts)
        self._ms = (time.perf_counter() - t0) * 1000.0

        self.pub_count.publish(Int32(data=len(clusters)))
        if self.vis and clusters:
            self.pub_markers.publish(self._markers(clusters, msg.header.stamp))

        nearest = self._nearest_in_corridor(clusters)
        if nearest is None:
            self.pub_nearest.publish(Float32MultiArray(data=[]))
        else:
            x, y, half_w, dist, n = nearest
            self.pub_nearest.publish(Float32MultiArray(
                data=[float(x), float(y), float(half_w), float(dist), float(n)]))

        now = time.time()
        if now - self._last_log > 1.5:
            self._last_log = now
            if nearest:
                self.get_logger().info(
                    f"簇 {len(clusters)} 个，最近障碍 前方 {nearest[3]:.1f}m "
                    f"(y={nearest[1]:+.2f}m, 半宽 {nearest[2]:.2f}m, {int(nearest[4])} 点) "
                    f"[{self._ms:.1f}ms]")
            else:
                self.get_logger().info(
                    f"簇 {len(clusters)} 个，走廊内无障碍 [{self._ms:.1f}ms]")

    @staticmethod
    def _cloud_to_xyz(msg: PointCloud2) -> Optional[np.ndarray]:
        try:
            arr = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        except Exception:  # noqa: BLE001
            return None
        if arr is None or arr.size == 0:
            return None
        return np.asarray(arr, dtype=np.float32).reshape(-1, 3)

    def _cluster(self, pts: np.ndarray) -> List[Tuple[float, float, float, float, int]]:
        """返回 [(cx, cy, 半宽, 最近距离, 点数), ...]，按距离升序。"""
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        keep = ((x > self.roi_x_min) & (x < self.roi_x_max) &
                (np.abs(y) < self.roi_y) &
                (z > self.ground_z) & (z < self.top_z))
        p = pts[keep]
        if p.shape[0] < self.min_points:
            return []

        v = self.voxel
        keys = np.floor(p[:, :3] / v).astype(np.int64)
        uniq, inverse = np.unique(keys, axis=0, return_inverse=True)
        ds = DisjointSet(uniq.shape[0])
        index = {(int(uniq[i, 0]), int(uniq[i, 1]), int(uniq[i, 2])): i
                 for i in range(uniq.shape[0])}
        for i in range(uniq.shape[0]):
            kx, ky, kz = int(uniq[i, 0]), int(uniq[i, 1]), int(uniq[i, 2])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        if dx == 0 and dy == 0 and dz == 0:
                            continue
                        j = index.get((kx + dx, ky + dy, kz + dz))
                        if j is not None:
                            ds.union(i, j)

        inv = np.asarray(inverse).reshape(-1)
        roots = np.array([ds.find(int(r)) for r in inv])
        out: List[Tuple[float, float, float, float, int]] = []
        for root in np.unique(roots):
            sel = p[roots == root]
            if sel.shape[0] < self.min_points:
                continue
            # 大块的东西（楼、墙、树篱）是"场景"不是"障碍物"。
            # 不滤掉的话，"走廊内最近障碍"经常切到路边建筑，
            # 避障就会对着墙打方向。
            if (sel[:, 1].max() - sel[:, 1].min()) > self.max_cluster_width:
                continue
            out.append((float(sel[:, 0].mean()),
                        float(sel[:, 1].mean()),
                        float((sel[:, 1].max() - sel[:, 1].min()) / 2.0),
                        float(np.hypot(sel[:, 0], sel[:, 1]).min()),
                        int(sel.shape[0])))
        out.sort(key=lambda c: c[3])
        return out[:self.max_clusters]

    def _nearest_in_corridor(self, clusters):
        """只有横向真的压到我们车道上的才算"挡路"。

        注意这里用"障碍物最近的横向边缘" |y|-hw < corridor 作为判据，
        并且按纵向距离 x 排序取最近的一个。
        早先版本是按径向距离取最近，结果远处一根路边的杆子会把
        真正挡在路中间的路障顶掉，避障方向一会儿左一会儿右。
        """
        in_lane = [c for c in clusters if abs(c[1]) - c[2] < self.corridor]
        if not in_lane:
            return None
        return min(in_lane, key=lambda c: c[0])

    def _markers(self, clusters, stamp) -> MarkerArray:
        arr = MarkerArray()
        for i, (cx, cy, half_w, dist, _n) in enumerate(clusters):
            m = Marker()
            m.header.frame_id = "base_link"
            m.header.stamp = stamp
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.orientation.w = 1.0
            m.scale.x = 3.0
            m.scale.y = max(0.5, half_w * 2.0)
            m.scale.z = 1.6
            t = min(1.0, dist / 30.0)
            m.color.r, m.color.g = 1.0 - 0.7 * t, 0.15 + 0.7 * t
            m.color.b, m.color.a = 0.15, 0.45
            arr.markers.append(m)
        return arr


def main() -> None:
    rclpy.init()
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
