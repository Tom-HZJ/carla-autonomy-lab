#!/usr/bin/env python3
"""M6: 运动学模型预测控制（MPC）。

跟 M5 的纯跟踪比，MPC 显式优化未来 N 步的横向 / 航向 / 速度误差，
所以弯道里不会像纯跟踪那样"切内道"。

模型：自行车模型（后轴参考点）
    x'     = x + v·cos(θ)·dt
    y'     = y + v·sin(θ)·dt
    θ'     = θ + v/L·tan(δ)·dt
    v'     = v + a·dt
状态 [x, y, θ, v]，控制 [a, δ]。

代价：Σ w_lat·e_lat² + w_yaw·e_yaw² + w_v·(v-v_ref)² + w_δ·δ² + w_a·a²

输入:  /carla/ego/odometry
输出:  /carla/ego/vehicle_control   (linear.x=目标速度, angular.z=前轮转角)
      /carla/ego/cross_track_error
      /carla/ego/global_path, /carla/ego/lookahead_marker
"""

from __future__ import annotations

import math
import time
from typing import List, Optional, Tuple

import casadi as ca
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker

from carla_autonomy import route as route_utils


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class MpcSolver:
    """把优化问题建一次，之后每次只喂初始状态和参考轨迹。"""

    def __init__(self, n: int = 12, dt: float = 0.1, wheelbase: float = 2.9,
                 max_steer: float = 0.7, v_max: float = 12.0,
                 a_max: float = 2.5, a_min: float = -5.0,
                 a_lat_max: float = 2.5, a_lat_hard: float = 5.0,
                 w_lat: float = 12.0, w_yaw: float = 8.0, w_v: float = 2.0,
                 w_delta: float = 1.0, w_a: float = 0.2) -> None:
        self.n, self.dt, self.L = n, dt, wheelbase
        self.max_steer = max_steer
        self.a_lat_max = a_lat_max      # 用来算"舒适过弯速度"
        self.a_lat_hard = a_lat_hard    # 用来算转角硬上限（比舒适值宽松）
        opti = ca.Opti()
        self.opti = opti
        self.X = opti.variable(4, n + 1)
        self.U = opti.variable(2, n)
        self.x0 = opti.parameter(4)
        self.ref = opti.parameter(4, n + 1)
        # 每个阶段的转角硬上限：a_lat = v²·tan(δ)/L ≤ a_lat_hard
        #   → tan(δ) ≤ a_lat_hard·L/v²
        # 注意这里用的是 a_lat_hard（更宽松），而不是限速用的 a_lat_max：
        # 限速要"保守"，但真出了横向误差时得允许它狠狠打方向修回来。
        # 之前两者共用一个值，结果弯里一旦有偏差就打不回来，直接跑丢。
        self.steer_lim = opti.parameter(n + 1)

        opti.subject_to(self.X[:, 0] == self.x0)
        for k in range(n):
            x, y, th, v = self.X[0, k], self.X[1, k], self.X[2, k], self.X[3, k]
            a, d = self.U[0, k], self.U[1, k]
            opti.subject_to(self.X[0, k + 1] == x + v * ca.cos(th) * dt)
            opti.subject_to(self.X[1, k + 1] == y + v * ca.sin(th) * dt)
            opti.subject_to(self.X[2, k + 1] == th + v / self.L * ca.tan(d) * dt)
            opti.subject_to(self.X[3, k + 1] == v + a * dt)
            opti.subject_to(opti.bounded(-max_steer, d, max_steer))
            opti.subject_to(opti.bounded(-self.steer_lim[k], d, self.steer_lim[k]))
            opti.subject_to(opti.bounded(a_min, a, a_max))
            opti.subject_to(opti.bounded(0.0, self.X[3, k + 1], v_max))

        cost = 0
        for k in range(1, n + 1):
            x, y, th, v = self.X[0, k], self.X[1, k], self.X[2, k], self.X[3, k]
            xr, yr, thr, vr = self.ref[0, k], self.ref[1, k], self.ref[2, k], self.ref[3, k]
            e_lat = -ca.sin(thr) * (x - xr) + ca.cos(thr) * (y - yr)
            e_yaw = ca.atan2(ca.sin(th - thr), ca.cos(th - thr))
            cost += w_lat * e_lat ** 2 + w_yaw * e_yaw ** 2 + w_v * (v - vr) ** 2
        for k in range(n):
            cost += w_delta * self.U[1, k] ** 2 + w_a * self.U[0, k] ** 2
        opti.minimize(cost)

        opts = {"print_time": False,
                "ipopt": {"print_level": 0, "max_iter": 60, "tol": 1e-3,
                          "warm_start_init_point": "yes", "sb": "yes"}}
        opti.solver("ipopt", opts)
        self._warm: Optional[list] = None

    def solve(self, x0: List[float], ref) -> Optional[Tuple[float, float, float]]:
        """返回 (a, delta, v_plan)。v_plan 是前瞻若干步之后的速度目标。

        为什么不直接用第 1 步的速度：模型里加速度有上限（2.5 m/s²），
        第 1 步只能把速度抬 0.25 m/s，把这个数当目标速度发给下层，
        车就只能以 0.25 m/s 的速度往上爬，几乎不动。
        所以取前瞻 k 步的速度当"速度目标"，让下层 PID 去追——
        这样 MPC 规划的加减速曲线能被真正执行出来。
        """
        # 注意：嵌套 list 不能直接喂给 Opti.set_value，要先转成 DM
        self.opti.set_value(self.x0, ca.DM(x0))
        self.opti.set_value(self.ref, ca.DM(ref))
        v_refs = [ref[3][k] for k in range(self.n + 1)]
        lims = []
        for k in range(self.n + 1):
            v = max(v_refs[k], 1.0)
            lims.append(min(self.max_steer,
                            math.atan(self.a_lat_hard * self.L / (v * v))))
        self.opti.set_value(self.steer_lim, ca.DM(lims))
        if self._warm is not None:
            self.opti.set_initial(self.X, self._warm[0])
            self.opti.set_initial(self.U, self._warm[1])
        try:
            sol = self.opti.solve()
        except RuntimeError:
            self._warm = None
            self.status = "FAIL"
            return None
        stats = sol.stats()
        self.status = (f"{stats.get('return_status', '?')}/it={stats.get('iter_count', '?')}"
                       f"/obj={stats.get('iterations', {}).get('inf_pr', 0) if isinstance(stats.get('iterations'), dict) else ''}")
        X = sol.value(self.X)
        U = sol.value(self.U)
        self._warm = [X, U]
        k = max(1, self.n // 4)
        return float(U[0, 0]), float(U[1, 0]), float(X[3, k])


class MpcController(Node):
    def __init__(self) -> None:
        super().__init__("mpc_controller")

        self.declare_parameter("target_speed", 8.0)
        self.declare_parameter("horizon", 20)
        self.declare_parameter("dt", 0.1)
        self.declare_parameter("wheelbase", 2.9)
        self.declare_parameter("max_steer_rad", 0.7)
        self.declare_parameter("v_max", 12.0)
        self.declare_parameter("route_file", "")
        self.declare_parameter("spawn_index", 0)
        self.declare_parameter("route_step", 2.0)
        self.declare_parameter("a_lat_max", 2.5)
        self.declare_parameter("a_lat_hard", 5.0)
        self.declare_parameter("go", True)
        self.declare_parameter("solve_hz", 20.0)
        self.declare_parameter("debug_frames", 0)
        self.declare_parameter("debug_period", 0.0)   # >0 时每 N 秒无条件打一条

        self.target_speed = float(self.get_parameter("target_speed").value)
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.max_steer = float(self.get_parameter("max_steer_rad").value)
        self.go = bool(self.get_parameter("go").value)
        self._dbg = int(self.get_parameter("debug_frames").value)
        self._dbg_i = 0
        self._dbg_period = float(self.get_parameter("debug_period").value)
        self._dbg_last = 0.0

        self.mpc = MpcSolver(n=int(self.get_parameter("horizon").value),
                             dt=float(self.get_parameter("dt").value),
                             wheelbase=self.wheelbase,
                             max_steer=self.max_steer,
                             v_max=float(self.get_parameter("v_max").value),
                             a_lat_max=float(self.get_parameter("a_lat_max").value),
                             a_lat_hard=float(self.get_parameter("a_lat_hard").value))

        self.path: List[Tuple[float, float]] = []
        self._load_route()
        self._cum: List[float] = [0.0]
        for i in range(1, len(self.path)):
            self._cum.append(self._cum[-1] + math.dist(self.path[i - 1], self.path[i]))
        self._total = self._cum[-1]
        self._curv = self._compute_curvature()

        self.nearest_idx = 0
        self.cross_track = 0.0
        self._last_steer = 0.0
        self._last_speed = 0.0
        self._solve_ms = 0.0

        self.pub_ctrl = self.create_publisher(Twist, "/carla/ego/vehicle_control", 10)
        self.pub_path = self.create_publisher(Path, "/carla/ego/global_path", 10)
        self.pub_marker = self.create_publisher(Marker, "/carla/ego/lookahead_marker", 10)
        self.pub_auto = self.create_publisher(Bool, "/carla/ego/set_autopilot", 10)
        self.pub_xte = self.create_publisher(Float32, "/carla/ego/cross_track_error", 10)
        self.create_subscription(Odometry, "/carla/ego/odometry", self.on_odom, 10)
        self.create_subscription(Bool, "/carla/ego/go", self.on_go, 10)
        self.publish_path()
        self.create_timer(2.0, self.publish_path)

        self.get_logger().info(
            f"MPC 就绪：N={self.mpc.n} dt={self.mpc.dt} 路径 {len(self.path)} 点 / "
            f"{route_utils.path_length(self.path):.0f} m，巡航 {self.target_speed:.1f} m/s")

    # ------------------------------------------------------------------ 路线
    def _load_route(self) -> None:
        rf = self.get_parameter("route_file").value
        if rf and __import__("os").path.isfile(rf):
            self.path = route_utils.load(rf)
            self.get_logger().info(f"从文件加载路线: {rf}")
            return
        import carla
        client = carla.Client("localhost", 2000)
        client.set_timeout(30.0)
        world = client.get_world()
        sp = world.get_map().get_spawn_points()
        idx = int(self.get_parameter("spawn_index").value) % len(sp)
        raw = route_utils.build_lane_loop(world.get_map(), sp[idx].location,
                                          step=float(self.get_parameter("route_step").value))
        closed = route_utils.close_loop(raw)
        self.path = route_utils.resample(
            route_utils.smooth(closed, window=3, iterations=1, closed=True), step=1.0)
        self.get_logger().info(f"现场生成路线 {len(self.path)} 点")

    def publish_path(self) -> None:
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    # ------------------------------------------------------------------ 参考轨迹
    def _nearest_index(self, xy) -> int:
        n = len(self.path)
        best_i, best_d = self.nearest_idx, float("inf")
        span = max(30, n // 10)
        for k in range(-10, span):
            i = (self.nearest_idx + k) % n
            d = math.dist(xy, self.path[i])
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def _compute_curvature(self) -> List[float]:
        """逐点曲率 κ = 1/R，用相邻三点外接圆算。"""
        n = len(self.path)
        kappa = [0.0] * n
        for i in range(n):
            a, b, c = self.path[(i - 1) % n], self.path[i], self.path[(i + 1) % n]
            ab = math.dist(a, b)
            bc = math.dist(b, c)
            ca = math.dist(c, a)
            cross = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
            area2 = cross          # 2 * 三角形面积
            if area2 < 1e-9 or ab * bc * ca < 1e-9:
                kappa[i] = 0.0
            else:
                kappa[i] = 2.0 * area2 / (ab * bc * ca)
        # 轻微平滑，避免单点噪声导致限速抖动
        return [(kappa[i - 1] + 2.0 * kappa[i] + kappa[(i + 1) % n]) / 4.0
                for i in range(n)]

    def _point_at(self, idx: int, s: float):
        """沿路径从 idx 往前走 s 米，返回 (x, y, theta, kappa)。"""
        n = len(self.path)
        cur = idx
        remain = s
        for _ in range(n):
            nxt = (cur + 1) % n
            seg = math.dist(self.path[cur], self.path[nxt])
            if seg < 1e-6:
                cur = nxt
                continue
            if remain <= seg:
                t = remain / seg
                a, b = self.path[cur], self.path[nxt]
                x = a[0] + (b[0] - a[0]) * t
                y = a[1] + (b[1] - a[1]) * t
                k = self._curv[cur] * (1 - t) + self._curv[nxt] * t
                return x, y, math.atan2(b[1] - a[1], b[0] - a[0]), k
            remain -= seg
            cur = nxt
        a, b = self.path[cur], self.path[(cur + 1) % n]
        return a[0], a[1], math.atan2(b[1] - a[1], b[0] - a[0]), self._curv[cur]

    def _build_ref(self, idx: int, speed: float):
        n = self.mpc.n
        ref = [[0.0] * (n + 1) for _ in range(4)]
        for k in range(n + 1):
            s = speed * self.mpc.dt * k
            x, y, th, kappa = self._point_at(idx, s)
            ref[0][k], ref[1][k], ref[2][k] = x, y, th
            ref[3][k] = self._speed_profile(kappa, speed)
        return ref

    def _speed_profile(self, kappa: float, base: float) -> float:
        """按曲率限速：v ≤ sqrt(a_lat_max / κ)，这就是"弯前减速"的来源。"""
        if kappa < 1e-4:
            return base
        return min(base, math.sqrt(self.mpc.a_lat_max / kappa))

    # ------------------------------------------------------------------ 回调
    def on_go(self, msg: Bool) -> None:
        self.go = bool(msg.data)

    def on_odom(self, msg: Odometry) -> None:
        if not self.path:
            return
        p = msg.pose.pose.position
        xy = (p.x, p.y)
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

        idx = self._nearest_index(xy)
        self.nearest_idx = idx
        self.cross_track = math.dist(xy, self.path[idx])
        self.pub_xte.publish(Float32(data=float(self.cross_track)))

        if not self.go:
            self.pub_ctrl.publish(Twist())
            return

        ref = self._build_ref(idx, self.target_speed)
        t0 = time.perf_counter()
        out = self.mpc.solve([xy[0], xy[1], yaw, v], ref)
        self._solve_ms = (time.perf_counter() - t0) * 1000.0

        if out is None:
            # 求解失败就退回上一次的指令，避免车突然停住
            steer, speed_cmd = self._last_steer, self._last_speed
        else:
            acc, steer, v_next = out
            del acc
            speed_cmd = max(0.0, min(self.target_speed, v_next))
            self._last_steer, self._last_speed = steer, speed_cmd

        m = Twist()
        m.linear.x = speed_cmd
        m.angular.z = steer
        self.pub_ctrl.publish(m)

        if self._dbg and self._dbg_i < self._dbg:
            self._dbg_i += 1
            self.get_logger().info(
                f"[dbg{self._dbg_i}] xy=({xy[0]:.2f},{xy[1]:.2f}) xte={self.cross_track:.3f} "
                f"steer={math.degrees(steer):.1f}deg v={speed_cmd:.1f} solve={self._solve_ms:.1f}ms "
                f"status={getattr(self.mpc, 'status', '?')}")
        if self._dbg_period > 0 and time.time() - self._dbg_last >= self._dbg_period:
            self._dbg_last = time.time()
            xr, yr, thr, _ = self._point_at(idx, 0.0)
            xf, yf, thf, _ = self._point_at(idx, 12.0)
            self.get_logger().info(
                f"[t] idx={idx} xte={self.cross_track:.2f} yaw={math.degrees(yaw):6.1f} "
                f"ref_yaw_now={math.degrees(thr):6.1f} ref_yaw_12m={math.degrees(thf):6.1f} "
                f"steer={math.degrees(steer):6.1f} v={speed_cmd:.1f}")

        mk = Marker()
        mk.header.frame_id = "map"
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.type = Marker.SPHERE
        mk.action = Marker.ADD
        mk.scale.x = mk.scale.y = mk.scale.z = 1.5
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.2, 0.6, 1.0, 0.9
        tx, ty, _, _ = self._point_at(idx, max(6.0, 1.0 * self.target_speed))
        mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = tx, ty, 1.0
        self.pub_marker.publish(mk)


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = MpcController()
        node.pub_auto.publish(Bool(data=False))
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
