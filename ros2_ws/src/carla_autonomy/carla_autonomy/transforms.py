"""CARLA 坐标系 <-> ROS 坐标系 的转换。

这是整个工程最容易踩坑的地方，先在这里一次性说清楚：

    CARLA (Unreal) : 左手系，X 前, Y 右, Z 上, 角度制
    ROS (REP-103)  : 右手系, X 前, Y 左, Z 上, 弧度制

两者的关系是一个镜像变换：
        S = diag(1, -1, 1)
    p_ros = S @ p_carla
    R_ros = S @ R_carla @ S

注意角速度是「轴矢量」，镜像下多一个负号：
    w_ros = -S @ w_carla = (-wx, +wy, -wz)

姿态我们不自己拼欧拉角（CARLA 的 pitch/roll 正方向很容易搞错），
而是直接问 CARLA 要 forward / right / up 三个向量，拼出旋转矩阵再转四元数，
这样无论 CARLA 内部用什么欧拉约定都不会错。
"""

from __future__ import annotations

import math
from typing import Tuple

S_SIGN = (1.0, -1.0, 1.0)


def carla_location_to_ros(location) -> Tuple[float, float, float]:
    """(x, y, z) -> (x, -y, z)"""
    return (location.x, -location.y, location.z)


def carla_vector_to_ros(vector) -> Tuple[float, float, float]:
    """线速度、加速度这类真矢量：v_ros = S v_carla"""
    return (vector.x, -vector.y, vector.z)


def carla_angular_velocity_to_ros(vector) -> Tuple[float, float, float]:
    """角速度是轴矢量：w_ros = -S w_carla"""
    return (-vector.x, vector.y, -vector.z)


def quaternion_from_matrix(m) -> Tuple[float, float, float, float]:
    """3x3 旋转矩阵 -> 四元数 (x, y, z, w)。用 Shepperd 方法，数值稳定。"""
    m00, m01, m02 = m[0][0], m[0][1], m[0][2]
    m10, m11, m12 = m[1][0], m[1][1], m[1][2]
    m20, m21, m22 = m[2][0], m[2][1], m[2][2]
    trace = m00 + m11 + m22

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    return (x, y, z, w)


def carla_rotation_to_ros_quaternion(rotation) -> Tuple[float, float, float, float]:
    """carla.Rotation -> ROS 四元数 (x, y, z, w)"""
    fwd = rotation.get_forward_vector()
    right = rotation.get_right_vector()
    up = rotation.get_up_vector()

    # 旋转矩阵的列 = 三个基向量在 CARLA 系下的坐标
    r = [
        [fwd.x, right.x, up.x],
        [fwd.y, right.y, up.y],
        [fwd.z, right.z, up.z],
    ]
    s = S_SIGN
    # R_ros = S R_carla S
    r_ros = [
        [s[i] * r[i][j] * s[j] for j in range(3)]
        for i in range(3)
    ]
    return quaternion_from_matrix(r_ros)


def carla_transform_to_ros_pose(transform):
    """carla.Transform -> (position(x,y,z), orientation(x,y,z,w))"""
    return (carla_location_to_ros(transform.location),
            carla_rotation_to_ros_quaternion(transform.rotation))


def yaw_from_ros_quaternion(q) -> float:
    """从四元数取 yaw（rad，绕 ROS Z 轴，逆时针为正）"""
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def carla_yaw_deg(actor) -> float:
    """车辆在 CARLA 里的航向角（度）。ROS 下 yaw_rad = -radians(这个值)"""
    return actor.get_transform().rotation.yaw

