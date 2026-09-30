#!/usr/bin/env python3
"""M5: 纯跟踪（Pure Pursuit）+ 速度规划。

输入:  /carla/ego/odometry                （自车位姿）
输出:  /carla/ego/vehicle_control         （Twist: linear.x=目标速度, angular.z=前轮转角）
      /carla/ego/global_path             （nav_msgs/Path，给 RViz 看）
      /carla/ego/lookahead_marker        （visualization_msgs/Marker，纯跟踪目标点）

路线来源: 启动时用 CARLA 地图现场生成一条沿车道中心线的闭环
（也可以用 route_file 参数从文件加载）。

用法:
    ros2 run carla_autonomy pure_pursuit --ros-args -p target_speed:=9.0
"""

from __future__ import annotations

import math
import os
import sys
import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray
from visualization_msgs.msg import Marker

from carla_autonomy import route as route_utils


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PurePursuit(Node):
    def __init__(self) -> None:
        super().__init__("pure_pursuit")

        self.declare_parameter("target_speed", 8.0)          # m/s（巡航速度，实测调优值）
        self.declare_parameter("min_speed", 3.0)
        self.declare_parameter("wheelbase", 2.9)             # Tesla Model 3 大致轴距
        self.declare_parameter("lookahead_gain", 0.45)       # Ld = gain * v + bias
        self.declare_parameter("lookahead_bias", 1.5)
        self.declare_parameter("lookahead_min", 4.0)
        self.declare_parameter("lookahead_max", 10.0)
        self.declare_parameter("max_steer_rad", 0.7)
        self.declare_parameter("cross_track_gain", 1.8)   # Stanley 式的横向误差反馈
        self.declare_parameter("cross_track_limit", 0.15)  # 该反馈最多贡献多少 rad
        self.declare_parameter("route_file", "")
        self.declare_parameter("route_step", 2.0)
        self.declare_parameter("spawn_index", 0)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("go", True)
        self.declare_parameter("debug_frames", 0)   # >0 时打印前 N 帧的计算细节
        # ---- 避障参数 ----
        self.declare_parameter("avoid_enable", True)
        self.declare_parameter("avoid_clearance", 1.1)   # 绕行时给障碍物留的横向余量(米)
        self.declare_parameter("avoid_max_offset", 3.2)  # 最大横向绕行量
        self.declare_parameter("avoid_lookahead", 26.0)  # 多远开始准备绕
        self.declare_parameter("slow_distance", 26.0)    # 多远开始减速
        self.declare_parameter("stop_distance", 5.0)     # 多近必须刹停
        self.declare_parameter("ego_half_width", 0.95)   # 自车半宽，用来算绕行要多少余量
        self._dbg = int(self.get_parameter("debug_frames").value)
        self._dbg_i = 0
        self.avoid_on = bool(self.get_parameter("avoid_enable").value)
        self.avoid_clear = float(self.get_parameter("avoid_clearance").value)
        self.avoid_max = float(self.get_parameter("avoid_max_offset").value)
        self.avoid_look = float(self.get_parameter("avoid_lookahead").value)
        self.slow_d = float(self.get_parameter("slow_distance").value)
        self.stop_d = float(self.get_parameter("stop_distance").value)
        self.ego_half_width = float(self.get_parameter("ego_half_width").value)
        self.obstacle = None            # (x, y, half_w, dist, points)
        self.avoid_offset = 0.0
        self._stuck_since = 0.0
        self._recover_until = 0.0
        self._avoid_side = 0        # 0=没锁定, +1=从左边绕, -1=从右边绕
        self._side_hold = 0.0       # 锁定还剩多少秒

        self.target_speed = float(self.get_parameter("target_speed").value)
        self.min_speed = float(self.get_parameter("min_speed").value)
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.ld_gain = float(self.get_parameter("lookahead_gain").value)
        self.ld_bias = float(self.get_parameter("lookahead_bias").value)
        self.ld_min = float(self.get_parameter("lookahead_min").value)
        self.ld_max = float(self.get_parameter("lookahead_max").value)
        self.max_steer = float(self.get_parameter("max_steer_rad").value)
        self.ct_gain = float(self.get_parameter("cross_track_gain").value)
        self.ct_limit = float(self.get_parameter("cross_track_limit").value)
        self.go = bool(self.get_parameter("go").value)

        self.path: List[Tuple[float, float]] = []
        self.nearest_idx = 0
        self.lap_count = 0
        self._prev_idx = 0

        self._load_route()

        self.pub_ctrl = self.create_publisher(Twist, "/carla/ego/vehicle_control", 10)
        self.pub_path = self.create_publisher(Path, "/carla/ego/global_path", 10)
        self.pub_marker = self.create_publisher(Marker, "/carla/ego/lookahead_marker", 10)
        self.pub_auto = self.create_publisher(Bool, "/carla/ego/set_autopilot", 10)
        self.pub_xte = self.create_publisher(Float32, "/carla/ego/cross_track_error", 10)
        self.create_subscription(Odometry, "/carla/ego/odometry", self.on_odom, 10)
        self.create_subscription(Bool, "/carla/ego/go", self.on_go, 10)
        self.create_subscription(Float32MultiArray, "/carla/ego/nearest_obstacle",
                                 self.on_obstacle, 10)
        # M8: 如果 ST-Graph 规划器在跑，就用它算出来的目标速度（时空联合规划），
        # 否则退回本节点自己的几何避障减速。
        self.st_speed = None
        self.st_time = 0.0
        self.create_subscription(Float32, "/carla/ego/st_speed_cmd", self.on_st_speed, 10)

        self.publish_path()
        # 控制循环由 odometry 回调驱动（桥那边是 20Hz），不需要额外定时器
        # 但路径要在启动后也一直重发：RViz 往往是后连上来的，
        # 只发一次（VOLATILE）它会看不到。
        self.create_timer(2.0, self.publish_path)
        self.get_logger().info(
            f"pure pursuit 就绪：路径 {len(self.path)} 点 / {route_utils.path_length(self.path):.0f} m，"
            f"巡航 {self.target_speed:.1f} m/s")

    # ------------------------------------------------------------------ 路线
    def _load_route(self) -> None:
        route_file = self.get_parameter("route_file").value
        if route_file and os.path.isfile(route_file):
            self.path = route_utils.load(route_file)
            self.get_logger().info(f"从文件加载路线: {route_file}")
            return

        import carla
        client = carla.Client("localhost", 2000)
        client.set_timeout(30.0)
        world = client.get_world()
        spawn_points = world.get_map().get_spawn_points()
        idx = int(self.get_parameter("spawn_index").value) % len(spawn_points)
        start = spawn_points[idx].location
        step = float(self.get_parameter("route_step").value)
        self.get_logger().info(f"沿车道生成路线，起点 spawn[{idx}] ...")
        raw = route_utils.build_lane_loop(world.get_map(), start, step=step)
        self.path = route_utils.resample(raw, step=1.0)
        self.get_logger().info(
            f"生成 {len(raw)} 个车道点 -> 重采样 {len(self.path)} 点，"
            f"长度 {route_utils.path_length(self.path):.0f} m，"
            f"闭环={route_utils.is_closed(self.path)}")

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

    # ------------------------------------------------------------------ 回调
    def on_go(self, msg: Bool) -> None:
        self.go = bool(msg.data)

    def on_obstacle(self, msg: Float32MultiArray) -> None:
        self.obstacle = tuple(msg.data) if len(msg.data) >= 5 else None

    def on_st_speed(self, msg: Float32) -> None:
        self.st_speed = float(msg.data)
        self.st_time = time.time()

    def on_odom(self, msg: Odometry) -> None:
        if not self.path:
            return
        pos = msg.pose.pose.position
        xy = (pos.x, pos.y)
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

        idx = self._nearest_index(xy)
        # 圈数统计（路径回到起点附近）
        if self._prev_idx > len(self.path) * 0.9 and idx < len(self.path) * 0.1:
            self.lap_count += 1
            self.get_logger().info(f"完成第 {self.lap_count} 圈")
        self._prev_idx = idx
        self.nearest_idx = idx

        # 横向误差（车到最近路径点的距离），用于验收
        self.cross_track = math.dist(xy, self.path[idx])
        self.pub_xte.publish(Float32(data=float(self.cross_track)))

        if not self.go:
            t = Twist()
            self.pub_ctrl.publish(t)
            return

        # ---- 卡死自救：想出弯但车不动，就倒一小段重新找角度 ----
        # 真实自驾栈基本都有这个逻辑。没有它的话，一旦贴上障碍物，
        # 纯跟踪只会一直往前顶，永远出不来。
        now = time.time()
        if speed < 0.25 and self.target_speed > 1.0:
            if self._stuck_since == 0.0:
                self._stuck_since = now
        else:
            self._stuck_since = 0.0
        if self._stuck_since and now - self._stuck_since > 2.5:
            self.get_logger().warn("检测到卡死，倒车重新找角度")
            self._recover_until = now + 2.0
            self._stuck_since = 0.0
        if now < self._recover_until:
            rec = Twist()
            rec.linear.x = -3.0
            rec.angular.z = -0.35 if self.avoid_offset > 0 else 0.35
            self.pub_ctrl.publish(rec)
            return

        ld = max(self.ld_min, min(self.ld_max,
                                  self.ld_gain * max(speed, 3.0) + self.ld_bias))
        target = self._lookahead_point(idx, ld)
        if target is None:
            return

        # ---- 避障：把前瞻点横向挪开，从旁边绕过去 ----
        speed_scale, offset = self._avoid_command()
        if abs(offset) > 1e-3:
            th = self._path_heading(idx)
            target = (target[0] - math.sin(th) * offset,   # 左法向 = (-sin, cos)
                      target[1] + math.cos(th) * offset)

        dx = target[0] - xy[0]
        dy = target[1] - xy[1]
        # 目标点转到车体坐标系
        cy, sy = math.cos(-yaw), math.sin(-yaw)
        lx = cy * dx - sy * dy
        ly = sy * dx + cy * dy
        dist = math.hypot(lx, ly)

        steer = 0.0
        if dist > 0.1:
            alpha = math.atan2(ly, lx)
            steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha), dist)

        # 纯跟踪在弯里会稳定地"切内道"（稳态横向偏差 ≈ Ld²/8R），
        # 再叠一个横向误差反馈把它压回去：delta -= atan(k·e / v)
        # 注意符号：e>0 表示车在路径左侧，此时应该往右打（ROS 里是负转角）。
        e = self._signed_cross_track(xy, idx)
        ct = math.atan2(-self.ct_gain * e, max(speed, 2.0))
        ct = max(-self.ct_limit, min(self.ct_limit, ct))
        steer = max(-self.max_steer, min(self.max_steer, steer + ct))

        # 转弯减速：转角越大目标速度越低
        curve = abs(steer) / self.max_steer
        v = self.target_speed * (1.0 - 0.65 * curve) * speed_scale
        v = max(self.min_speed, v)
        if speed_scale < 0.05:
            v = 0.0                      # 该刹停了

        # M8: ST-Graph 在线就用它给的速度
        # （它已经把障碍物在时间维度上的占位算进去了，比纯几何避障更准）
        if self.st_speed is not None and time.time() - self.st_time < 0.5:
            v = min(v, self.st_speed)

        msg_out = Twist()
        msg_out.linear.x = v
        msg_out.angular.z = steer
        self.pub_ctrl.publish(msg_out)

        if self.obstacle is not None and self.obstacle[3] < self.avoid_look:
            self.get_logger().info(
                f"[避障] 前方 {self.obstacle[3]:.1f}m (y={self.obstacle[1]:+.2f}) "
                f"-> 绕行 {offset:+.2f}m, 速度 x{speed_scale:.2f} = {v:.1f} m/s",
                throttle_duration_sec=1.0)

        if self._dbg and self._dbg_i < self._dbg:
            self._dbg_i += 1
            self.get_logger().info(
                f"[dbg{self._dbg_i}] xy=({xy[0]:.2f},{xy[1]:.2f}) yaw={math.degrees(yaw):.1f} "
                f"idx={idx} xte={self.cross_track:.2f} ld={ld:.1f} "
                f"target=({target[0]:.2f},{target[1]:.2f}) body=({lx:.2f},{ly:.2f}) "
                f"dist={dist:.2f} steer={math.degrees(steer):.1f}deg v={v:.1f}")

        self._publish_marker(target, steer, v)

    # ------------------------------------------------------------------ 工具
    def _nearest_index(self, xy: Tuple[float, float]) -> int:
        """在上一帧索引附近局部搜索，避免全程扫描，也避免抄近道。"""
        n = len(self.path)
        best_i, best_d = self.nearest_idx, float("inf")
        window = max(30, n // 10)
        for k in range(-10, window):
            i = (self.nearest_idx + k) % n
            d = math.dist(xy, self.path[i])
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def _path_heading(self, idx: int) -> float:
        n = len(self.path)
        a, b = self.path[idx], self.path[(idx + 1) % n]
        return math.atan2(b[1] - a[1], b[0] - a[0])

    def _avoid_command(self) -> Tuple[float, float]:
        """返回 (速度缩放 0~1, 横向绕行偏移)。

        关键点：**能绕就不停车**。
        早期版本只要进了 stop_distance 就把速度压到 0，结果车停在障碍物前面
        3 米处永远不动了 —— 停住之后就没有纵向运动，横向绕行也就无从谈起。
        所以这里先算"绕过去需要多少横向位移"：
          - 算得出来（不超过 avoid_max）-> 保持一个最低速度，边减速边绕；
          - 算不出来（两侧都不够）-> 才刹停。
        """
        if not self.avoid_on or self.obstacle is None:
            self.avoid_offset *= 0.9
            return 1.0, self.avoid_offset

        _ox, oy, half_w, dist, _n = self.obstacle
        if dist > self.avoid_look:
            self.avoid_offset *= 0.9
            return 1.0, self.avoid_offset

        # 障碍物占住 [oy-hw, oy+hw]，我们自己还有 half_ego + 余量。
        ego_half = self.ego_half_width
        need = half_w + ego_half + self.avoid_clear

        # 先看"保持现在的横向位置"能不能过：
        #   我们当前在 y≈0，只要 |oy| >= need 就说明本来就能过去，不用绕。
        # 这一步千万不能省 —— 少了它就会在"本来能过"的时候也硬指一个绕行点，
        # 结果等于主动往障碍物那边打方向（我之前就栽在这）。
        if abs(oy) >= need:
            want = 0.0
            blocked = False
            self._avoid_side = 0
        else:
            right = oy - need      # 从右边过，目标横向位置
            left = oy + need       # 从左边过
            # 一旦决定从哪边绕，就锁定一段时间（side_hold）。
            # 不锁的话，远处几个点测出来的 y 会左右飘，
            # 绕行方向跟着来回变，车就在原地画蛇。
            self._side_hold = max(0.0, self._side_hold - 0.05)
            if self._avoid_side == 0 or self._side_hold <= 0.0:
                self._avoid_side = -1 if abs(right) <= abs(left) else 1
                self._side_hold = 3.0
            if self._avoid_side < 0:
                want, blocked = right, abs(right) > self.avoid_max
            else:
                want, blocked = left, abs(left) > self.avoid_max

        # 渐入要快：固定 8m 过渡，而不是除以整个 avoid_look，
        # 否则 20m 外基本不绕，等看清楚已经贴脸了。
        # 4m 过渡就到位：太慢的话，等绕行量涨上来车已经贴上去了
        f = max(0.0, min(1.0, (self.avoid_look - dist) / 4.0))
        want = max(-self.avoid_max, min(self.avoid_max, want * f))
        self.avoid_offset = 0.75 * self.avoid_offset + 0.25 * want

        if dist >= self.slow_d:
            scale = 1.0
        else:
            t = max(0.0, (dist - self.stop_d) / max(0.1, self.slow_d - self.stop_d))
            scale = 0.25 + 0.75 * min(1.0, t)
        if blocked and dist <= self.slow_d:
            # 两侧都绕不过去：减速刹停
            scale = max(0.0, min(1.0, (dist - self.stop_d) / max(0.1, self.slow_d)))
        return scale, self.avoid_offset

    def _signed_cross_track(self, xy: Tuple[float, float], idx: int) -> float:
        """有符号横向误差：车在路径左侧为正。"""
        n = len(self.path)
        a = self.path[idx]
        b = self.path[(idx + 1) % n]
        theta = math.atan2(b[1] - a[1], b[0] - a[0])
        dx = xy[0] - a[0]
        dy = xy[1] - a[1]
        return -dx * math.sin(theta) + dy * math.cos(theta)

    def _lookahead_point(self, idx: int, ld: float) -> Optional[Tuple[float, float]]:
        n = len(self.path)
        acc = 0.0
        prev = self.path[idx]
        for k in range(1, n):
            i = (idx + k) % n
            cur = self.path[i]
            acc += math.dist(prev, cur)
            if acc >= ld:
                return cur
            prev = cur
        return None

    def _publish_marker(self, target, steer: float, v: float) -> None:
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 1.5
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.6, 0.0, 0.9
        m.pose.position.x = target[0]
        m.pose.position.y = target[1]
        m.pose.position.z = 1.0
        self.pub_marker.publish(m)


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = PurePursuit()
        # 先确保桥那边关掉 CARLA 内置自动驾驶
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
