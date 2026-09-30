#!/usr/bin/env python3
"""从 CARLA 地图生成一条沿车道中心线的闭环路线，存成 json。

用法:
    ros2 run carla_autonomy make_route --ros-args -p spawn_index:=0 \
        -p out:=/home/tom/Desktop/ROS2/ros2_ws/src/carla_autonomy/config/route_town10.json
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from carla_autonomy import route as route_utils


class MakeRoute(Node):
    def __init__(self) -> None:
        super().__init__("make_route")
        self.declare_parameter("spawn_index", 0)
        self.declare_parameter("step", 2.0)
        self.declare_parameter("out", "/home/tom/Desktop/ROS2/ros2_ws/src/"
                                     "carla_autonomy/config/route_town10.json")

    def run(self) -> int:
        import carla
        client = carla.Client("localhost", 2000)
        client.set_timeout(60.0)
        carla_map = client.get_world().get_map()

        spawn_index = int(self.get_parameter("spawn_index").value)
        step = float(self.get_parameter("step").value)
        out = self.get_parameter("out").value

        spawn_points = carla_map.get_spawn_points()
        spawn_index %= len(spawn_points)
        start = spawn_points[spawn_index].location
        self.get_logger().info(f"起点 spawn[{spawn_index}] = ({start.x:.1f}, {start.y:.1f})")

        raw = route_utils.build_lane_loop(carla_map, start, step=step)
        closed_raw = route_utils.close_loop(raw)
        smooth = route_utils.smooth(closed_raw, window=3, iterations=1, closed=True)
        path = route_utils.resample(smooth, step=1.0)
        length = route_utils.path_length(path)
        closed = route_utils.is_closed(path, tol=3.0)
        gap = math.dist(path[0], path[-1])
        self.get_logger().info(
            f"车道点 {len(raw)} -> 截断 {len(closed_raw)} -> 平滑 -> 重采样 {len(path)} 点，"
            f"长度 {length:.0f} m，接缝 {gap:.2f} m，闭环={closed}")

        route_utils.save(path, out, meta={"spawn_index": spawn_index,
                                          "length_m": round(length, 1),
                                          "closed": closed,
                                          "map": carla_map.name})
        self.get_logger().info(f"已保存 -> {out}")
        return 0


def main() -> None:
    rclpy.init()
    node = MakeRoute()
    code = 1
    try:
        code = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
