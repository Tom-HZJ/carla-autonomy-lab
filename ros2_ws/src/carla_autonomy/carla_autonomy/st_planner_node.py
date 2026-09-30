#!/usr/bin/env python3
"""M8: ST-Graph 时空联合规划。

跟前面几版"几何绕行"的根本区别：

    以前：看见障碍物 -> 算一个横向偏移绕过去（只看空间，不看时间）
    现在：把障碍物在一段时间内的占位投到 (s, t) 平面上，
          再在这张图上搜一条不撞的速度曲线（空间 + 时间一起算）

为什么这样更好：障碍物是**动的**，同一块空间在不同时间可能是空的。
只看空间的规划要么过分保守（一直刹），要么撞上。

实现：
    1. 状态量 s（沿路径走过的距离）、t（时间）、v（速度）
    2. 离散化：t 以 dt 分 N 步，v 以 dv 分 M 档
    3. 转移：s_{k+1} = s_k + v_k·dt，限制 |v_{k+1}-v_k| ≤ a_max·dt
    4. 碰撞：若 |s_k - s_obs(t_k)| < 安全距离，该状态不可行
    5. 代价：Σ (v - v_desired)² + w_a·a²  —— 既想快，又不想急加速
    6. 动态规划求最优 -> 回溯出 v(t) -> 发布目标速度

这是经典的 ST-Graph DP（Werling 等人的做法），不依赖任何外部库。

输入:  /carla/ego/odometry, /carla/ego/nearest_obstacle
输出:  /carla/ego/st_speed_cmd      (Float32, 目标速度 m/s)
      /carla/ego/st_profile        (Float32MultiArray, 规划出的 v(t) 曲线，给 RViz/调试)
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray


class StGraphPlanner(Node):
    def __init__(self) -> None:
        super().__init__("st_planner")

        # 规划时域 / 分辨率
        self.declare_parameter("horizon_s", 5.0)
        self.declare_parameter("dt", 0.2)
        self.declare_parameter("v_max", 12.0)
        self.declare_parameter("v_step", 0.5)
        self.declare_parameter("a_max", 2.0)        # 加速上限 m/s²
        self.declare_parameter("a_min", -4.5)       # 减速上限（刹车）
        # 障碍物参数
        self.declare_parameter("obs_length", 4.5)   # 障碍物纵向长度估计
        self.declare_parameter("ego_length", 4.7)
        self.declare_parameter("margin", 3.0)       # 额外安全余量
        self.declare_parameter("obstacle_speed_gain", 0.7)  # 障碍速度估计的平滑
        # 代价权重
        self.declare_parameter("w_speed", 1.0)
        self.declare_parameter("w_accel", 0.35)
        self.declare_parameter("w_jerk", 0.05)

        self.T = float(self.get_parameter("horizon_s").value)
        self.dt = float(self.get_parameter("dt").value)
        self.v_max = float(self.get_parameter("v_max").value)
        self.dv = float(self.get_parameter("v_step").value)
        self.a_max = float(self.get_parameter("a_max").value)
        self.a_min = float(self.get_parameter("a_min").value)
        self.obs_len = float(self.get_parameter("obs_length").value)
        self.ego_len = float(self.get_parameter("ego_length").value)
        self.margin = float(self.get_parameter("margin").value)
        self.k_obs = float(self.get_parameter("obstacle_speed_gain").value)
        self.w_speed = float(self.get_parameter("w_speed").value)
        self.w_accel = float(self.get_parameter("w_accel").value)

        self.Nt = int(round(self.T / self.dt))
        self.Mv = int(round(self.v_max / self.dv)) + 1

        self.speed = 0.0
        self.desired = 8.0                 # 由 /carla/ego/desired_speed 覆盖都可以
        self.obstacle: Optional[Tuple[float, float, float, float, float]] = None
        self._prev_obs_s: Optional[float] = None
        self._prev_obs_t = 0.0
        self.obs_speed = 0.0
        self._last_plan: List[float] = []

        self.pub_speed = self.create_publisher(Float32, "/carla/ego/st_speed_cmd", 10)
        self.pub_profile = self.create_publisher(Float32MultiArray, "/carla/ego/st_profile", 10)
        self.create_subscription(Odometry, "/carla/ego/odometry", self.on_odom, 10)
        self.create_subscription(Float32MultiArray, "/carla/ego/nearest_obstacle",
                                 self.on_obstacle, 10)
        self.create_subscription(Float32, "/carla/ego/desired_speed",
                                 self.on_desired, 10)

        self.get_logger().info(
            f"ST-Graph 规划器就绪：时域 {self.T:.1f}s ({self.Nt} 步 × {self.dt}s)，"
            f"速度 {self.Mv} 档，a∈[{self.a_min}, {self.a_max}]")

    # ------------------------------------------------------------------ 回调
    def on_desired(self, msg: Float32) -> None:
        self.desired = float(msg.data)

    def on_odom(self, msg: Odometry) -> None:
        v = msg.twist.twist
        self.speed = math.hypot(v.linear.x, v.linear.y)

    def on_obstacle(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 5:
            self.obstacle = None
            self._prev_obs_s = None
            return
        x, y, half_w, dist, pts = msg.data[:5]
        # 只关心挡在车道里的：横向半宽和自车宽度比
        if abs(y) - half_w > 1.4:
            self.obstacle = None
            self._prev_obs_s = None
            return
        now = self.get_clock().now().nanoseconds / 1e9
        # 用相邻两帧的距离变化估障碍物速度（相对速度 -> 绝对速度）
        if self._prev_obs_s is not None and now > self._prev_obs_t:
            dt = now - self._prev_obs_t
            if 1e-3 < dt < 1.0:
                closing = (self._prev_obs_s - dist) / dt      # 正 = 在靠近
                obs_v = self.speed - closing                  # 障碍物绝对速度
                self.obs_speed = (1 - self.k_obs) * self.obs_speed + self.k_obs * obs_v
                self.obs_speed = max(0.0, min(self.v_max, self.obs_speed))
        self._prev_obs_s, self._prev_obs_t = dist, now
        self.obstacle = (x, y, half_w, dist, pts)
        self.plan_and_publish()

    # ------------------------------------------------------------------ 规划
    def _blocked(self, s_ego: float, t: float) -> bool:
        """t 时刻自车走到 s_ego 会不会撞上障碍物。"""
        if self.obstacle is None:
            return False
        _x, _y, _hw, s0, _pts = self.obstacle
        s_obs = s0 + self.obs_speed * t          # 障碍物匀速外推
        clear = (self.obs_len + self.ego_len) / 2.0 + self.margin
        return abs(s_ego - s_obs) < clear

    def plan(self, desired: Optional[float] = None) -> List[float]:
        """三维 DP 求 v(t)，返回长度 Nt+1 的速度曲线（m/s）。

        状态必须是 (时间 k, 累计位移 s, 速度 v) 三个量：
        只索引 (k, v) 是不够的 —— 不同路径到同一个 (k, v) 时，
        走出来的累计位移 s 并不相同，碰撞判断就没法做。
        """
        if desired is None:
            desired = self.desired
        Nt, Mv, dt, dv = self.Nt, self.Mv, self.dt, self.dv
        ds = 0.5
        Ns = int(round(self.v_max * self.T / ds)) + 2
        INF = 1e9

        cost = np.full((Nt + 1, Ns, Mv), INF, dtype=np.float64)
        # 只记前驱的速度档和前驱位移档，回溯时够用
        pv = np.full((Nt + 1, Ns, Mv), -1, dtype=np.int32)
        ps = np.full((Nt + 1, Ns, Mv), -1, dtype=np.int32)

        v0 = int(round(min(self.speed, self.v_max) / dv))
        cost[0, 0, v0] = 0.0

        for k in range(Nt):
            t_k = k * dt
            for si in range(Ns):
                row = cost[k, si]
                if not np.any(row < INF):
                    continue
                s_k = si * ds
                for vi in range(Mv):
                    c = row[vi]
                    if c >= INF:
                        continue
                    v_i = vi * dv
                    for vj in range(Mv):
                        v_j = vj * dv
                        a = (v_j - v_i) / dt
                        if a > self.a_max or a < self.a_min:
                            continue
                        v_avg = 0.5 * (v_i + v_j)
                        s_next = s_k + v_avg * dt
                        if s_next > (Ns - 1) * ds:
                            continue
                        # 中途和终点都查一次，防止"一步跨过去"漏检
                        if self._blocked(0.5 * (s_k + s_next), t_k + 0.5 * dt):
                            continue
                        if self._blocked(s_next, t_k + dt):
                            continue
                        sj = int(round(s_next / ds))
                        add = (self.w_speed * (v_avg - desired) ** 2
                               + self.w_accel * a * a) * dt
                        if c + add < cost[k + 1, sj, vj]:
                            cost[k + 1, sj, vj] = c + add
                            pv[k + 1, sj, vj] = vi
                            ps[k + 1, sj, vj] = si

        flat = cost[Nt]
        if not np.any(flat < INF):
            # 整条时域都被堵死：退化成"用最大减速度刹停"
            return [max(0.0, self.speed + self.a_min * dt * (k + 1))
                    for k in range(Nt + 1)]

        si, vi = np.unravel_index(int(np.argmin(flat)), flat.shape)
        seq = [0.0] * (Nt + 1)
        for k in range(Nt, -1, -1):
            seq[k] = vi * dv
            if k > 0:
                si, vi = ps[k, si, vi], pv[k, si, vi]
                if si < 0 or vi < 0:
                    break
        return seq

    # ------------------------------------------------------------------ 多模态
    def score(self, prof: List[float]) -> Tuple[float, float]:
        """给一条候选速度曲线打分，返回 (总代价, 最小碰撞余量)。

        借 SparseDrive 的 **collision-aware rescore** 思路：
        碰撞风险要**单独重打分**，不能混在优化目标里被速度项稀释掉。
        DP 内部已经有碰撞约束，但它是在"期望速度"这一个意图下搜的；
        换一个意图（比如让行）搜出来的曲线，安全余量可能完全不同。
        """
        s, prev_v, cost = 0.0, self.speed, 0.0
        min_clear = 1e9
        for k, v in enumerate(prof):
            a = (v - prev_v) / self.dt
            cost += (self.w_speed * (v - self.desired) ** 2
                     + self.w_accel * a * a) * self.dt
            prev_v = v
            s += v * self.dt
            if self.obstacle is not None:
                _x, _y, _hw, s0, _pts = self.obstacle
                s_obs = s0 + self.obs_speed * (k * self.dt)
                clear = abs(s - s_obs) - (self.obs_len + self.ego_len) / 2.0
                min_clear = min(min_clear, clear)
        # 碰撞风险单独重加权：贴得越近代价涨得越猛
        if min_clear < 0:
            cost += 1e6 + 1e4 * (-min_clear)
        elif min_clear < self.margin:
            cost += 1e3 * (self.margin - min_clear) ** 2
        return cost, min_clear

    def plan_multi(self) -> Tuple[str, List[float]]:
        """生成 K 条不同意图的候选，重打分后选最安全的。"""
        cands = [
            ("保持", self.desired),                       # 维持巡航速度
            ("让行", min(self.speed * 0.5, self.desired * 0.4)),  # 主动减速
            ("抢行", min(self.v_max, self.desired * 1.35)),       # 加速通过
        ]
        best = None
        for name, want in cands:
            prof = self.plan(want)
            cost, clear = self.score(prof)
            if best is None or cost < best[0]:
                best = (cost, clear, name, prof)
        return best[2], best[3]

    def plan_and_publish(self) -> None:
        # 多模态：生成"保持/让行/抢行"三条候选，用碰撞感知重打分选最优
        # （思路来自 SparseDrive 的 hierarchical planning selection）
        intent, prof = self.plan_multi()
        self._last_plan = prof
        # 发给控制器的是"下一步该跑多快"
        cmd = prof[1] if len(prof) > 1 else self.desired
        self.pub_speed.publish(Float32(data=float(cmd)))
        self.pub_profile.publish(Float32MultiArray(data=[float(v) for v in prof[:25]]))
        self._last_intent = intent


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = StGraphPlanner()
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
