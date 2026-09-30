"""路线生成 / 读写。

不依赖 CARLA 的 agents 包（那个在发行包里，pip 的 wheel 不带），
直接用 map.get_waypoint() + waypoint.next() 沿车道中心线往前爬，
优先选同一个 lane_id 的后继，这样在路口会自然地"直行"，
一直爬到绕回起点附近，就得到一个闭环。

注意坐标系：这里存的是 **ROS 坐标系**（y 已经取反），
因为下游的纯跟踪控制器在 ROS 系里算误差。
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]


def carla_to_ros_xy(location) -> Point:
    return (location.x, -location.y)


def build_lane_loop(carla_map,
                    start_location,
                    step: float = 2.0,
                    max_points: int = 4000,
                    close_radius: float = 8.0,
                    min_points_before_close: int = 60) -> List[Point]:
    """从 start_location 所在车道出发，沿车道往前爬，回到起点附近就停。"""
    import carla  # 局部导入：本模块在没有 carla 的环境里也要能读路线文件
    wp = carla_map.get_waypoint(start_location,
                                project_to_road=True,
                                lane_type=carla.LaneType.Driving)
    if wp is None:
        raise RuntimeError("起点不在可行驶车道上")

    path: List[Point] = [carla_to_ros_xy(wp.transform.location)]
    start = path[0]

    for i in range(max_points):
        nxt = wp.next(step)
        if not nxt:
            break
        same_lane = [w for w in nxt if w.lane_id == wp.lane_id and w.road_id == wp.road_id]
        wp = same_lane[0] if same_lane else nxt[0]
        p = carla_to_ros_xy(wp.transform.location)
        path.append(p)
        if i >= min_points_before_close and math.dist(p, start) < close_radius:
            break

    return path


def path_length(path: Sequence[Point]) -> float:
    return sum(math.dist(path[i], path[i + 1]) for i in range(len(path) - 1))


def close_loop(path: List[Point], from_frac: float = 0.5) -> List[Point]:
    """剪掉回路尾巴，让"末点 -> 首点"这一跳尽可能短。

    沿车道爬出来的路径，最后一段往往停在离起点还有几米的地方，
    如果直接当闭环用，纯跟踪在接缝处会看到 8m 的突跳，车就甩出去了。
    这里从后半段里挑一个离起点最近的点，把后面的都扔掉。
    """
    if len(path) < 4:
        return path
    start = path[0]
    lo = int(len(path) * from_frac)
    best_j, best_d = len(path) - 1, math.dist(path[-1], start)
    for i in range(lo, len(path)):
        d = math.dist(path[i], start)
        if d < best_d:
            best_d, best_j = d, i
    trimmed = path[:best_j + 1]
    # 接缝还差得多的话，线性补几个点把洞填上
    gap = math.dist(trimmed[-1], start)
    if gap > 1.0:
        k = int(gap)  # 每米补一个点
        a = trimmed[-1]
        for m in range(1, k + 1):
            t = m / (k + 1)
            trimmed.append((a[0] + (start[0] - a[0]) * t,
                            a[1] + (start[1] - a[1]) * t))
    return trimmed


def smooth(path: Sequence[Point], window: int = 5, iterations: int = 2,
           closed: bool = True) -> List[Point]:
    """滑动平均，把路口处车道跳变造成的毛刺磨掉。"""
    pts = list(path)
    n = len(pts)
    if n < window + 2:
        return pts
    half = window // 2
    for _ in range(iterations):
        out = []
        for i in range(n):
            xs = ys = 0.0
            cnt = 0
            for k in range(-half, half + 1):
                j = i + k
                if closed:
                    j %= n
                elif j < 0 or j >= n:
                    continue
                xs += pts[j][0]
                ys += pts[j][1]
                cnt += 1
            out.append((xs / cnt, ys / cnt))
        pts = out
    return pts


def is_closed(path: Sequence[Point], tol: float = 12.0) -> bool:
    return len(path) > 2 and math.dist(path[0], path[-1]) < tol


def resample(path: Sequence[Point], step: float = 1.0) -> List[Point]:
    """等距重采样，纯跟踪用起来更稳。"""
    if len(path) < 2:
        return list(path)
    out = [path[0]]
    acc = 0.0
    prev = path[0]
    for p in path[1:]:
        d = math.dist(prev, p)
        if d == 0:
            continue
        # 沿这一段按 step 打点
        while acc + d >= step:
            remain = step - acc
            t = remain / d
            q = (prev[0] + (p[0] - prev[0]) * t, prev[1] + (p[1] - prev[1]) * t)
            out.append(q)
            prev = q
            d = math.dist(prev, p)
            acc = 0.0
        acc += d
        prev = p
    return out


def save(path: Sequence[Point], filename: str | Path, meta: Optional[dict] = None) -> None:
    data = {"points": [[round(x, 4), round(y, 4)] for x, y in path],
            "meta": meta or {}}
    Path(filename).write_text(json.dumps(data, ensure_ascii=False, indent=1))


def load(filename: str | Path) -> List[Point]:
    data = json.loads(Path(filename).read_text())
    if isinstance(data, dict):
        return [(float(a), float(b)) for a, b in data["points"]]
    return [(float(a), float(b)) for a, b in data]
