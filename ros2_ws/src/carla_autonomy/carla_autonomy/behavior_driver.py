#!/usr/bin/env python3
"""M7b: 用 CARLA 官方 agents 包驾驶（复用的开源实现）。

为什么要有这个节点：
  我们自己手写的纯跟踪 + LiDAR 聚类，避障一直过不去——
  绕行偏移加在"前瞻点"上，等障碍物进到 3~4m 才反应过来，横向来不及挪。

  而 CARLA 发行包自带的 PythonAPI/carla/agents 是官方维护的：
    - GlobalRoutePlanner   全局路线（车道拓扑图搜索）
    - LocalPlanner         局部规划、车道变换
    - BasicAgent           避障 + 红绿灯 + 路口让行
                           （避障用的是仿真真值 actor 列表 + 路线多边形判断，
                            比用 LiDAR 去猜稳得多）

  所以 M7 的策略改成「两条腿」：
    - 这条腿：复用官方 agent，把"雨夜避障 + 路口"的效果先跑出来；
    - 另一条腿：保留我们自研的 LiDAR 聚类（obstacle_detector），
      作为"感知演示 / 学习用"，两者可以对比。

用法:
    ros2 run carla_autonomy behavior_driver
    ros2 run carla_autonomy behavior_driver --ros-args -p target_speed_kmh:=35.0

注意: 起这个节点时不要同时起 pure_pursuit/mpc_controller，
桥检测到 0.5s 没有控制指令就会放手，让 agent 自己 apply_control。
"""

from __future__ import annotations

import math
import os
import sys
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


def _ensure_agents_on_path() -> bool:
    """把发行包里的 agents 目录接进 sys.path。

    注意：加的是 PythonAPI/carla 这一层，它下面只有 agents/ 和 dist/，
    没有 carla/ 子目录，所以不会把 pip 装的 carla 客户端顶掉。
    """
    home = os.environ.get("CARLA_HOME", "/home/tom/Desktop/ROS2/carla/CARLA_0.9.15")
    for cand in (os.path.join(home, "PythonAPI", "carla"),
                 os.path.join(home, "PythonAPI")):
        if os.path.isdir(os.path.join(cand, "agents")) or \
           os.path.isdir(os.path.join(cand, "carla", "agents")):
            if cand not in sys.path:
                sys.path.insert(0, cand)
            return True
    return False


class BehaviorDriver(Node):
    def __init__(self) -> None:
        super().__init__("behavior_driver")

        self.declare_parameter("target_speed_kmh", 30.0)
        self.declare_parameter("behavior", "normal")      # cautious | normal | aggressive
        self.declare_parameter("go", True)
        self.declare_parameter("opt_route", True)         # 用全局路线规划，而不是纯车道跟随

        self.go = bool(self.get_parameter("go").value)
        self._plan_sent = False
        self._lap_start = None

        if not _ensure_agents_on_path():
            raise RuntimeError("找不到 CARLA 的 agents 包，检查 CARLA_HOME")

        import carla
        from agents.navigation.behavior_agent import BehaviorAgent
        self._carla = carla

        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(30.0)
        self.world = self.client.get_world()

        self.ego = self._find_ego()
        if self.ego is None:
            raise RuntimeError("没找到 ego_vehicle，先把 carla_bridge 起起来")
        self.get_logger().info(f"接管自车 id={self.ego.id}")

        speed = float(self.get_parameter("target_speed_kmh").value)
        behavior = str(self.get_parameter("behavior").value)
        self.agent = BehaviorAgent(self.ego, behavior=behavior,
                                   opt_dict={"target_speed": speed})
        self.agent.follow_speed_limits(False)
        self._set_destination()

        self.pub_path = self.create_publisher(Path, "/carla/ego/global_path", 10)
        self.pub_xte = self.create_publisher(Float32, "/carla/ego/cross_track_error", 10)
        self.create_subscription(Odometry, "/carla/ego/odometry", self.on_odom, 10)
        self.create_subscription(Bool, "/carla/ego/go", self.on_go, 10)

        self.nearest_idx = 0
        self.cross_track = 0.0
        self.get_logger().info(
            f"BehaviorAgent 就绪：行为={behavior}，目标速度={speed:.0f} km/h")

    # ------------------------------------------------------------------ 工具
    def _find_ego(self):
        # 新连上来的客户端第一次 get_actors() 经常返回空，
        # 等一会儿再查就好（CARLA 的多客户端有连接建立延迟）。
        for attempt in range(20):
            try:
                self.world = self.client.get_world()
                for a in self.world.get_actors().filter("vehicle.*"):
                    if a.attributes.get("role_name") == "ego_vehicle":
                        return a
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"查询世界失败（第 {attempt + 1} 次）: {exc}")
            time.sleep(1.0)
        return None

    def _set_destination(self) -> None:
        """挑一个离自车足够远的 spawn 点当终点。"""
        spawns = self.world.get_map().get_spawn_points()
        loc = self.ego.get_transform().location
        best, best_d = None, -1.0
        for sp in spawns:
            d = math.hypot(sp.location.x - loc.x, sp.location.y - loc.y)
            if d > best_d:
                best, best_d = sp, d
        if best is not None:
            self.agent.set_destination(best.location)
            self.get_logger().info(
                f"目的地 ({best.location.x:.1f}, {best.location.y:.1f})，直线距离 {best_d:.0f} m")

    def on_go(self, msg: Bool) -> None:
        self.go = bool(msg.data)

    # ------------------------------------------------------------------ 主循环
    def on_odom(self, msg: Odometry) -> None:
        if not self.go:
            return
        try:
            control = self.agent.run_step()
            self.ego.apply_control(control)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"run_step 失败: {exc}", throttle_duration_sec=2.0)
            return

        # 把 agent 规划出来的路线发出去，方便 RViz 里看
        if not self._plan_sent:
            self._publish_plan()
            self._plan_sent = True

        # 横向误差：自车到最近规划点的距离（跟 M5/M6 用同一套指标口径）
        try:
            plan = list(self.agent._local_planner.get_plan())
            if plan:
                p = msg.pose.pose.position
                d = [math.hypot(w.transform.location.x - p.x,
                                -(w.transform.location.y) - p.y) for w, _ in plan]
                self.cross_track = min(d)
                self.pub_xte.publish(Float32(data=float(self.cross_track)))
        except Exception:  # noqa: BLE001
            pass

    def _publish_plan(self) -> None:
        try:
            plan = list(self.agent._local_planner.get_plan())
        except Exception:  # noqa: BLE001
            return
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for wp, _opt in plan:
            ps = PoseStamped()
            ps.header = msg.header
            # CARLA -> ROS: y 取反
            ps.pose.position.x = wp.transform.location.x
            ps.pose.position.y = -wp.transform.location.y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)
        self.get_logger().info(f"发出规划路径 {len(msg.poses)} 个点")


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = BehaviorDriver()
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
