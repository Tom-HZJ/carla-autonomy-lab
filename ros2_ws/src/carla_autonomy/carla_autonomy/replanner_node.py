#!/usr/bin/env python3
"""M8 上层：堵死了就重规划（迷宫鼠那套）。

为什么需要它：
    下层 ST-Graph 只能在**给定路径上**做速度规划。障碍物把车道堵死时，
    它唯一能给的答案就是 v=0 —— 车停在原地，永远过不去。

    迷宫鼠遇到死路的做法是：标记走不通 -> 从当前位置重新找路 -> 走新路。
    这个节点就干"重新找路"这件事。

逻辑：
    1. 盯着 /carla/ego/nearest_obstacle，如果本车道被堵且持续 N 秒 -> 触发
    2. 看障碍物偏在哪边，往**空的那边**借一条车道（默认让 3.5m = 一个车道宽）
    3. 在基础路径上叠一个"横向鼓包"：从当前位置开始平滑地偏出去、
       绕过障碍物之后再平滑地并回来
    4. 把这条新路径发到 /carla/ego/route_override，纯跟踪会切过去

    另外记录"刚试过哪边" —— 迷宫鼠不会立刻回头再撞同一堵墙，
    我们也不要在左右车道之间来回横跳。

输入:  /carla/ego/odometry, /carla/ego/nearest_obstacle
输出:  /carla/ego/route_override (nav_msgs/Path)
      /carla/ego/replan_count    (Int32)
"""

from __future__ import annotations

import math
import os
import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32

from carla_autonomy import route as route_utils


class Replanner(Node):
    def __init__(self) -> None:
        super().__init__("replanner")

        self.declare_parameter("route_file",
                               "/home/tom/Desktop/ROS2/ros2_ws/src/carla_autonomy/"
                               "config/route_town10.json")
        self.declare_parameter("blocked_hold_s", 1.5)   # 堵多久算"真堵了"
        self.declare_parameter("trigger_dist", 28.0)    # 多远开始考虑重规划
        self.declare_parameter("lane_width", 3.5)       # 借道的横向距离
        self.declare_parameter("shift_len", 26.0)       # 鼓包的总长度（米）
        # 和 obstacle_detector / st_planner 用同一个门限（自车半宽 + 余量）
        self.declare_parameter("min_clear", 1.45)
        self.declare_parameter("retry_cooldown_s", 12.0)  # 同一边多久内不重试
        self.declare_parameter("forward_points", 140)   # 新路径往前取多少个点

        self.blocked_hold = float(self.get_parameter("blocked_hold_s").value)
        self.trigger_dist = float(self.get_parameter("trigger_dist").value)
        self.lane_w = float(self.get_parameter("lane_width").value)
        self.shift_len = float(self.get_parameter("shift_len").value)
        self.min_clear = float(self.get_parameter("min_clear").value)
        self.cooldown = float(self.get_parameter("retry_cooldown_s").value)
        self.fwd = int(self.get_parameter("forward_points").value)

        rf = str(self.get_parameter("route_file").value)
        self.base: List[Tuple[float, float]] = []
        if os.path.isfile(rf):
            self.base = route_utils.load(rf)
        self.get_logger().info(f"基础路径 {len(self.base)} 点")

        self.xy: Optional[Tuple[float, float]] = None
        self.yaw = 0.0
        self.speed = 0.0
        self.obstacle: Optional[Tuple[float, ...]] = None
        self._blocked_since = 0.0
        self._last_side = 0
        self._last_try: dict = {}
        self._count = 0

        self.pub_path = self.create_publisher(Path, "/carla/ego/route_override", 10)
        self.pub_count = self.create_publisher(Int32, "/carla/ego/replan_count", 10)
        self.create_subscription(Odometry, "/carla/ego/odometry", self.on_odom, 10)
        self.create_subscription(Float32MultiArray, "/carla/ego/nearest_obstacle",
                                 self.on_obstacle, 10)
        self.create_timer(0.2, self.tick)
        self.get_logger().info(
            f"重规划器就绪：堵 {self.blocked_hold:.1f}s 触发，借道 {self.lane_w:.1f}m，"
            f"同侧重试冷却 {self.cooldown:.0f}s")

    # ------------------------------------------------------------------
    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.xy = (p.x, p.y)
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        v = msg.twist.twist
        self.speed = math.hypot(v.linear.x, v.linear.y)

    def on_obstacle(self, msg: Float32MultiArray) -> None:
        self.obstacle = tuple(msg.data) if len(msg.data) >= 5 else None

    # ------------------------------------------------------------------
    def tick(self) -> None:
        now = time.time()
        # 1. 判断是不是真堵了：本车道有障碍、离得够近、而且车已经快停了
        # 触发条件只看**感知**：本车道上有障碍、而且已经够近。
        # 千万不要加"车速 < 1.0"这种条件 —— 那等于要求"下层先把我刹停，
        # 我才去重规划"，两层的耦合就是这么自己造出来的：
        # 没有下层刹车时，重规划一次都不会触发（实测 0 次）。
        # 判断"路堵了"是感知的事，不是下层执行器的事。
        blocked = (self.obstacle is not None
                   and self.obstacle[3] < self.trigger_dist
                   and abs(self.obstacle[1]) - self.obstacle[2] < self.min_clear)
        if blocked:
            if self._blocked_since == 0.0:
                self._blocked_since = now
        else:
            self._blocked_since = 0.0
            return
        if now - self._blocked_since < self.blocked_hold:
            return

        # 2. 选往哪边借道：障碍物偏左就往右借，反之亦然
        oy = self.obstacle[1]
        prefer = -1 if oy >= 0 else 1          # -1 = 往右(y-)借
        sides = [prefer, -prefer]
        for side in sides:
            if now - self._last_try.get(side, 0.0) < self.cooldown:
                continue                    # 刚试过这边，别撞第二次
            self._last_try[side] = now
            if self._replan(side):
                self._blocked_since = 0.0
                self._count += 1
                self.pub_count.publish(Int32(data=self._count))
                self.get_logger().warn(
                    f"[重规划] 前方 {self.obstacle[3]:.1f}m 堵死，"
                    f"往{'左' if side > 0 else '右'}借 {self.lane_w:.1f}m "
                    f"(第 {self._count} 次)")
                return
        # 两边都试过还不行
        self.get_logger().error("[重规划] 左右都试过了，还是过不去", throttle_duration_sec=5.0)

    # ------------------------------------------------------------------
    def _nearest_idx(self) -> int:
        if not self.base or self.xy is None:
            return -1
        best, bd = 0, 1e18
        for i, p in enumerate(self.base):
            d = (p[0] - self.xy[0]) ** 2 + (p[1] - self.xy[1]) ** 2
            if d < bd:
                bd, best = d, i
        return best

    def _replan(self, side: int) -> bool:
        """在基础路径上叠一个横向鼓包，返回是否成功。"""
        if len(self.base) < self.fwd + 10 or self.xy is None:
            return False
        n = len(self.base)
        i0 = self._nearest_idx()
        if i0 < 0:
            return False

        pts: List[Tuple[float, float]] = []
        ramp = max(6, int(self.shift_len * 0.45))     # 渐变的点数
        flat = max(4, self.fwd - 2 * ramp)
        bump = 2 * ramp + flat
        # 关键：下发的是**整条闭环**，不是前面那一小段。
        # 之前只发 140 个点，纯跟踪的 lookahead 是按模 n 取点的，
        # 车一进这条 140 点的路径就再也跑不出去，只能在这一小段上原地打转，
        # 下一圈的障碍物永远遇不到 —— 迷宫鼠绕过去之后必须回到原路。
        # 所以：前 bump 个点带横向鼓包（绕行），后面全部偏移为 0（回到原路线）。
        total = len(self.base)
        for k in range(total):
            idx = (i0 + k) % n
            x, y = self.base[idx]
            # 前一段线性升到满偏移，中间保持，后一段线性收回来
            if k < ramp:
                f = k / ramp
            elif k < ramp + flat:
                f = 1.0
            else:
                f = max(0.0, 1.0 - (k - ramp - flat) / ramp)
            # 鼓包方向：沿路径左法向
            j = (idx + 1) % n
            nx, ny = self.base[j]
            th = math.atan2(ny - y, nx - x)
            off = side * self.lane_w * f
            pts.append((x - math.sin(th) * off, y + math.cos(th) * off))

        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)
        return True


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = Replanner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
