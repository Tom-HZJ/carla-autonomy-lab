#!/usr/bin/env python3
"""CARLA <-> ROS 2 桥梁节点（自研轻量版）。

这个节点同时扮演两个角色：
  1. 仿真管理：连 CARLA、设天气、生成自车、挂传感器、按固定步长推进世界；
  2. 数据桥接：把 CARLA 传感器转成 ROS 2 话题，并把控制指令写回车辆。

话题（默认命名空间 /carla/ego）:
  发布:
    ~/camera/image_raw        sensor_msgs/Image        (bgra8)
    ~/camera/camera_info      sensor_msgs/CameraInfo
    ~/lidar/points            sensor_msgs/PointCloud2  (x,y,z,intensity)
    ~/imu                     sensor_msgs/Imu
    ~/gnss                    sensor_msgs/NavSatFix
    ~/odometry                nav_msgs/Odometry
    /tf                       map -> base_link
  订阅:
    ~/vehicle_control         geometry_msgs/Twist
        linear.x  = 目标速度 (m/s)
        angular.z = 前轮转角 (rad)
      速度跟踪的 PID 放在本节点里（属于执行器层），
      上层控制器（pure pursuit / MPC）只管给运动学指令。
    ~/set_autopilot           std_msgs/Bool

用法:
    ros2 run carla_autonomy carla_bridge --ros-args -p spawn_ego:=true -p weather:=rain_night
"""

from __future__ import annotations

import math
import sys
import threading
import time
from array import array

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import (CameraInfo, Image, Imu, NavSatFix, NavSatStatus,
                             PointCloud2, PointField)
from std_msgs.msg import Bool
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

import carla

from carla_autonomy.transforms import (carla_angular_velocity_to_ros,
                                       carla_location_to_ros,
                                       carla_rotation_to_ros_quaternion,
                                       carla_vector_to_ros)


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


def rain_night() -> carla.WeatherParameters:
    w = carla.WeatherParameters()
    w.cloudiness = 95.0
    w.precipitation = 90.0
    w.precipitation_deposits = 90.0
    w.wind_intensity = 45.0
    w.fog_density = 30.0
    w.fog_distance = 80.0
    w.wetness = 95.0
    w.scattering_intensity = 2.0
    w.mie_scattering_scale = 0.5
    w.rayleigh_scattering_scale = 0.3
    w.sun_altitude_angle = -12.0
    w.sun_azimuth_angle = 200.0
    return w


def clear_noon() -> carla.WeatherParameters:
    w = carla.WeatherParameters()
    w.sun_altitude_angle = 60.0
    return w


WEATHERS = {
    "rain_night": rain_night,
    "clear": clear_noon,
}


class CarlaBridge(Node):
    def __init__(self) -> None:
        super().__init__("carla_bridge")

        # ---- 参数 ----
        self.declare_parameter("host", "localhost")
        self.declare_parameter("port", 2000)
        self.declare_parameter("town", "")              # 空 = 沿用当前地图
        self.declare_parameter("weather", "rain_night")  # rain_night | clear | keep
        self.declare_parameter("spawn_ego", True)
        self.declare_parameter("ego_filter", "vehicle.tesla.model3")
        self.declare_parameter("autopilot", False)
        self.declare_parameter("sync_mode", True)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("camera_width", 1280)
        self.declare_parameter("camera_height", 720)
        self.declare_parameter("lidar_channels", 64)
        self.declare_parameter("lidar_range", 80.0)
        self.declare_parameter("lidar_hz", 20.0)
        self.declare_parameter("lidar_mount_z", 2.5)   # LiDAR 相对 base_link 的高度
        self.declare_parameter("publish_camera", True)
        self.declare_parameter("publish_lidar", True)
        self.declare_parameter("follow_spectator", True)
        # CARLA 的 VehicleControl.steer ∈ [-1,1] 对应的前轮转角，实测标定出来的：
        #   steer=+0.10 -> 5.69°, steer=+0.20 -> 8.86°, steer=+0.45 -> 24.40°
        # 拟合出来 ≈ 1.0 rad/unit（不是 0.7！）。
        # 这里填错的话，上层算出的转角会被放大 1/0.7 ≈ 1.43 倍再执行，
        # 表现为"控制器以为打 12°、车实际打了 17°"，
        # 结果就是 MPC 的模型和真车对不上，弯里慢慢发散最后跑丢。
        self.declare_parameter("max_steer_rad", 1.0)
        self.declare_parameter("namespace", "/carla/ego")
        self.declare_parameter("spawn_index", 0)
        # 纵向速度 PID（执行器层）
        self.declare_parameter("speed_kp", 0.35)
        self.declare_parameter("speed_ki", 0.15)
        self.declare_parameter("speed_kd", 0.0)
        self.declare_parameter("max_throttle", 1.0)
        self.declare_parameter("max_brake", 1.0)

        self.host = self.get_parameter("host").value
        self.port = self.get_parameter("port").value
        self.town = self.get_parameter("town").value
        self.weather_name = self.get_parameter("weather").value
        self.spawn_ego = self.get_parameter("spawn_ego").value
        self.ego_filter = self.get_parameter("ego_filter").value
        self.sync_mode = self.get_parameter("sync_mode").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.max_steer = float(self.get_parameter("max_steer_rad").value)
        self.follow_spectator = bool(self.get_parameter("follow_spectator").value)
        self.speed_kp = float(self.get_parameter("speed_kp").value)
        self.speed_ki = float(self.get_parameter("speed_ki").value)
        self.speed_kd = float(self.get_parameter("speed_kd").value)
        self.max_throttle = float(self.get_parameter("max_throttle").value)
        self.max_brake = float(self.get_parameter("max_brake").value)

        self.actors: list = []
        self.ego: carla.Actor | None = None
        self._prev_settings = None
        self._closed = False
        self._lock = threading.Lock()
        self._control = carla.VehicleControl()
        self._autopilot = bool(self.get_parameter("autopilot").value)
        self._target_speed = 0.0
        self._steer = 0.0
        self._last_cmd_time = 0.0
        self._speed_err_sum = 0.0
        self._speed_err_prev = 0.0
        # 订阅者探测（每秒刷一次，避免在 CARLA 回调里频繁问 DDS）
        self._has_sub = {"camera": False, "lidar": False, "imu": False, "gnss": False}
        self._stats = {k: 0 for k in ("tick", "cam_cb", "cam_pub", "lidar_cb",
                                      "lidar_pub", "imu_cb", "gnss_cb")}
        self._cam_ms = 0.0

        # ---- 连 CARLA ----
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(30.0)
        if self.town:
            self.get_logger().info(f"加载地图 {self.town} ...")
            self.world = self.client.load_world(self.town)
        else:
            self.world = self.client.get_world()
        self.get_logger().info(
            f"已连接 CARLA {self.client.get_server_version()} / 地图 {self.world.get_map().name}")

        if self.weather_name in WEATHERS:
            self.world.set_weather(WEATHERS[self.weather_name]())
            self.get_logger().info(f"天气 = {self.weather_name}")

        self._setup_sync()
        self._setup_ego()
        self._setup_sensors()
        self._setup_ros()

        self.create_timer(1.0 / self.rate_hz, self.on_tick)
        self.create_timer(1.0, self._log_stats)
        self.get_logger().info("carla_bridge 就绪")

    # ------------------------------------------------------------------ 仿真
    def _setup_sync(self) -> None:
        settings = self.world.get_settings()
        self._prev_settings = (settings.synchronous_mode, settings.fixed_delta_seconds)
        if self.sync_mode:
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 1.0 / self.rate_hz
            self.world.apply_settings(settings)
            self.get_logger().info(
                f"同步模式开启，步长 {settings.fixed_delta_seconds:.3f}s ({self.rate_hz:.0f} Hz)")
        else:
            settings.synchronous_mode = False
            self.world.apply_settings(settings)

    def _setup_ego(self) -> None:
        # 上次异常退出可能留下没清理掉的 ego_vehicle，先把它们收掉，
        # 否则会复用到一台"跑到别处去了"的车，路线对不上。
        leftovers = [a for a in self.world.get_actors().filter("vehicle.*")
                     if a.attributes.get("role_name") == "ego_vehicle"]
        if leftovers and self.spawn_ego:
            for a in leftovers:
                self.get_logger().warn(f"清理上次残留的自车 id={a.id}")
                a.destroy()
            leftovers = []
        for actor in leftovers:
            self.ego = actor
            self.get_logger().info(f"复用已有自车 id={actor.id}")
            break

        if self.ego is None:
            if not self.spawn_ego:
                self.get_logger().warn("没找到 ego_vehicle，且 spawn_ego=false")
                return
            bp = self.world.get_blueprint_library().filter(self.ego_filter)[0]
            if bp.has_attribute("role_name"):
                bp.set_attribute("role_name", "ego_vehicle")
            spawn_points = self.world.get_map().get_spawn_points()
            # 首选 spawn_index 指定的点，但那儿可能被别的东西占了
            # （比如测试时摆的路障），那就顺着往后找第一个空的。
            # 没这个兜底的话，一次 "collision at spawn position"
            # 整个桥就直接起不来了。
            start = int(self.get_parameter("spawn_index").value) % len(spawn_points)
            order = [spawn_points[(start + k) % len(spawn_points)]
                     for k in range(len(spawn_points))]
            self.ego = None
            for sp in order:
                try:
                    self.ego = self.world.spawn_actor(bp, sp)
                    break
                except RuntimeError:
                    continue
            if self.ego is None:
                raise RuntimeError("所有 spawn 点都被占了，生成不了自车")
            self.actors.append(self.ego)
            self.get_logger().info(f"生成自车 {bp.id} id={self.ego.id}")

        if self._autopilot:
            tm = self.client.get_trafficmanager(8000)
            tm.set_synchronous_mode(self.sync_mode)
            self.ego.set_autopilot(True, 8000)
            self.get_logger().info("CARLA 内置自动驾驶已开启")

    def _setup_sensors(self) -> None:
        if self.ego is None:
            return
        bp_lib = self.world.get_blueprint_library()
        w = int(self.get_parameter("camera_width").value)
        h = int(self.get_parameter("camera_height").value)
        channels = int(self.get_parameter("lidar_channels").value)
        lidar_range = float(self.get_parameter("lidar_range").value)
        lidar_hz = float(self.get_parameter("lidar_hz").value)

        if self.get_parameter("publish_camera").value:
            cbp = bp_lib.find("sensor.camera.rgb")
            cbp.set_attribute("image_size_x", str(w))
            cbp.set_attribute("image_size_y", str(h))
            cbp.set_attribute("fov", "90")
            self.camera = self.world.spawn_actor(
                cbp, carla.Transform(carla.Location(x=1.5, z=2.4)), attach_to=self.ego)
            self.camera.listen(self._on_camera)
            self.actors.append(self.camera)
            self.get_logger().info(f"RGB 相机 {w}x{h}")
        else:
            self.camera = None

        if self.get_parameter("publish_lidar").value:
            lbp = bp_lib.find("sensor.lidar.ray_cast")
            lbp.set_attribute("channels", str(channels))
            lbp.set_attribute("range", str(lidar_range))
            lbp.set_attribute("rotation_frequency", str(lidar_hz))
            # 点密度直接决定"多远能发现小障碍物"。32 线 64 万点/秒时，
            # 20m 外的路障只有个位数点，成不了簇，避障根本来不及反应。
            lbp.set_attribute("points_per_second", str(int(channels * 30000)))
            self.lidar = self.world.spawn_actor(
                lbp, carla.Transform(
                    carla.Location(x=0.0, z=float(self.get_parameter("lidar_mount_z").value))),
                attach_to=self.ego)
            self.lidar.listen(self._on_lidar)
            self.actors.append(self.lidar)
            self.get_logger().info(f"LiDAR {channels} 线 @ {lidar_hz}Hz")
        else:
            self.lidar = None

        self.imu = self.world.spawn_actor(
            bp_lib.find("sensor.other.imu"),
            carla.Transform(carla.Location(x=0.0, z=1.0)), attach_to=self.ego)
        self.imu.listen(self._on_imu)
        self.actors.append(self.imu)

        self.gnss = self.world.spawn_actor(
            bp_lib.find("sensor.other.gnss"),
            carla.Transform(carla.Location(x=0.0, z=1.0)), attach_to=self.ego)
        self.gnss.listen(self._on_gnss)
        self.actors.append(self.gnss)
        self.get_logger().info("IMU + GNSS 已挂载")

    # -------------------------------------------------------------------- ROS
    def _setup_ros(self) -> None:
        ns = self.get_parameter("namespace").value
        self.pub_cam = self.create_publisher(Image, f"{ns}/camera/image_raw", SENSOR_QOS)
        self.pub_cam_info = self.create_publisher(CameraInfo, f"{ns}/camera/camera_info", SENSOR_QOS)
        self.pub_lidar = self.create_publisher(PointCloud2, f"{ns}/lidar/points", SENSOR_QOS)
        self.pub_imu = self.create_publisher(Imu, f"{ns}/imu", SENSOR_QOS)
        self.pub_gnss = self.create_publisher(NavSatFix, f"{ns}/gnss", SENSOR_QOS)
        self.pub_odom = self.create_publisher(Odometry, f"{ns}/odometry", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist, f"{ns}/vehicle_control", self._on_cmd, 10)
        self.create_subscription(Bool, f"{ns}/set_autopilot", self._on_autopilot, 10)

    def now_msg_header(self, frame_id: str):
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = frame_id
        return h

    # --------------------------------------------------------------- 回调/发布
    def _on_cmd(self, msg: Twist) -> None:
        # 符号约定（已实测）：
        #   ROS 侧：yaw 逆时针为正，转角正 = 左转
        #   CARLA 侧：VehicleControl.steer 正 = 右转
        # 两边正好相反，所以这里要取反。忘了取反的表现是车一转弯就
        # 正反馈失控画圈（角度越大误差越大 → 打得更死）。
        ros_steer = max(-1.0, min(1.0, float(msg.angular.z) / self.max_steer))
        with self._lock:
            self._target_speed = float(msg.linear.x)
            self._steer = -ros_steer
            self._last_cmd_time = time.time()
            if self._autopilot:
                self._autopilot = False
                if self.ego is not None and self.ego.is_alive:
                    self.ego.set_autopilot(False)

    def _speed_control(self, dt: float) -> carla.VehicleControl:
        """纵向 PID：给定目标速度，算出 throttle / brake。"""
        with self._lock:
            target = self._target_speed
            steer = self._steer
        speed = 0.0
        if self.ego is not None and self.ego.is_alive:
            v = self.ego.get_velocity()
            speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

        # 目标速度为负 = 倒车。CARLA 里倒车要用 gear=-1 + 正油门，
        # 靠 PID 出负油门是倒不了的（只会刹车），所以这里单独处理。
        if target < -0.1:
            err = (-target) - speed
            self._speed_err_sum = max(-5.0, min(5.0, self._speed_err_sum + err * dt))
            u = self.speed_kp * err + self.speed_ki * self._speed_err_sum
            return carla.VehicleControl(throttle=min(1.0, max(0.0, u)),
                                        steer=steer, brake=min(1.0, max(0.0, -u)),
                                        manual_gear_shift=True, gear=-1)

        err = target - speed
        self._speed_err_sum += err * dt
        self._speed_err_sum = max(-5.0, min(5.0, self._speed_err_sum))  # 抗积分饱和
        deriv = (err - self._speed_err_prev) / dt if dt > 0 else 0.0
        self._speed_err_prev = err

        u = self.speed_kp * err + self.speed_ki * self._speed_err_sum + self.speed_kd * deriv
        throttle = min(self.max_throttle, max(0.0, u))
        brake = min(self.max_brake, max(0.0, -u))
        return carla.VehicleControl(throttle=throttle, steer=steer, brake=brake)

    def _on_autopilot(self, msg: Bool) -> None:
        self._autopilot = bool(msg.data)
        if self.ego is None or not self.ego.is_alive:
            return
        tm = self.client.get_trafficmanager(8000)
        tm.set_synchronous_mode(self.sync_mode)
        self.ego.set_autopilot(self._autopilot, 8000)
        self.get_logger().info(f"autopilot = {self._autopilot}")

    def _on_camera(self, image: carla.Image) -> None:
        self._stats["cam_cb"] += 1
        if self._closed or not self._has_sub["camera"]:
            return
        t0 = time.perf_counter()
        msg = Image()
        msg.header = self.now_msg_header("camera")
        msg.height = image.height
        msg.width = image.width
        msg.encoding = "bgra8"
        msg.is_bigendian = False
        msg.step = image.width * 4
        t_raw0 = time.perf_counter()
        raw = bytes(image.raw_data)
        t_raw1 = time.perf_counter()
        # 注意：一定要传 array.array('B')。
        # 直接塞 bytes 会走 rosidl 生成代码里的 __debug__ 校验分支
        # （对每个元素跑两次 Python 级 all(...)），3.7MB 的图像要 ~200ms；
        # 传 array.array 则直接赋值，实测 ~1ms。
        msg.data = array("B", raw)
        t_asg = time.perf_counter()
        self.pub_cam.publish(msg)
        self._stats["cam_pub"] += 1
        t_pub = time.perf_counter()
        self._cam_ms = (t_pub - t0) * 1000.0
        self._cam_detail = (
            (t_raw1 - t_raw0) * 1000.0, (t_asg - t_raw1) * 1000.0, (t_pub - t_asg) * 1000.0)

        info = CameraInfo()
        info.header = msg.header
        info.height = image.height
        info.width = image.width
        info.distortion_model = "plumb_bob"
        fov = 90.0
        f = image.width / (2.0 * math.tan(math.radians(fov) / 2.0))
        info.k = [f, 0.0, image.width / 2.0, 0.0, f, image.height / 2.0, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, image.width / 2.0, 0.0, 0.0, f, image.height / 2.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self.pub_cam_info.publish(info)

    def _on_lidar(self, meas: carla.LidarMeasurement) -> None:
        self._stats["lidar_cb"] += 1
        if self._closed or not self._has_sub["lidar"]:
            return
        pts = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1, 4)
        # CARLA LiDAR 点是在"传感器系"里给的（x 前, y 右, z 上），
        # 而我们把 header.frame_id 写成了 base_link，所以要做两步换算：
        #   1) 加上安装高度（传感器在 base_link 上方 2.5m，地面点在传感器系里是 z≈-2.5，
        #      换算到 base_link 后应该落在 z≈0）
        #   2) y 取反（左手系 -> 右手系）
        # 符号写反的话，地面会跑到 z≈-5，RViz 里车像浮在半空，
        # 障碍物检测的地面/高度门限全错。
        pts = pts.copy()
        pts[:, 2] += float(self.get_parameter("lidar_mount_z").value)
        pts[:, 1] *= -1.0
        msg = PointCloud2()
        msg.header = self.now_msg_header("base_link")
        msg.height = 1
        msg.width = int(pts.shape[0])
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * msg.width
        msg.data = array("B", pts.astype(np.float32, copy=False).tobytes())
        msg.is_dense = True
        self.pub_lidar.publish(msg)
        self._stats["lidar_pub"] += 1

    def _on_imu(self, meas: carla.IMUMeasurement) -> None:
        self._stats["imu_cb"] += 1
        if self._closed or not self._has_sub["imu"]:
            return
        msg = Imu()
        msg.header = self.now_msg_header("base_link")
        a, g = meas.accelerometer, meas.gyroscope
        ax, ay, az = carla_vector_to_ros(a)
        gx, gy, gz = carla_angular_velocity_to_ros(g)
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        # 只用罗盘做 yaw，凑一个可用的姿态（CARLA compass 是相对正北的弧度）
        yaw = -meas.compass
        msg.orientation.z = math.sin(yaw / 2.0)
        msg.orientation.w = math.cos(yaw / 2.0)
        for i in (0, 1, 2):
            msg.orientation_covariance[i * 4] = 0.01
            msg.angular_velocity_covariance[i * 4] = 0.01
            msg.linear_acceleration_covariance[i * 4] = 0.01
        self.pub_imu.publish(msg)

    def _on_gnss(self, meas: carla.GNSSMeasurement) -> None:
        self._stats["gnss_cb"] += 1
        if self._closed or not self._has_sub["gnss"]:
            return
        msg = NavSatFix()
        msg.header = self.now_msg_header("gnss")
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = meas.latitude
        msg.longitude = meas.longitude
        msg.altitude = meas.altitude
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.pub_gnss.publish(msg)

    # ------------------------------------------------------------------- 主循环
    def _log_stats(self) -> None:
        if self._closed:
            return
        self._has_sub["camera"] = self.pub_cam.get_subscription_count() > 0
        self._has_sub["lidar"] = self.pub_lidar.get_subscription_count() > 0
        self._has_sub["imu"] = self.pub_imu.get_subscription_count() > 0
        self._has_sub["gnss"] = self.pub_gnss.get_subscription_count() > 0
        s = self._stats
        self.get_logger().info(
            f"1s: tick={s['tick']} cam cb/pub={s['cam_cb']}/{s['cam_pub']} "
            f"lidar cb/pub={s['lidar_cb']}/{s['lidar_pub']} "
            f"imu={s['imu_cb']} cam_pub_ms={self._cam_ms:.1f} "
            f"(raw/assign/pub={getattr(self, '_cam_detail', (0, 0, 0))[0]:.1f}/"
            f"{getattr(self, '_cam_detail', (0, 0, 0))[1]:.1f}/"
            f"{getattr(self, '_cam_detail', (0, 0, 0))[2]:.1f}) "
            f"subs={[k for k, v in self._has_sub.items() if v]}")
        for k in s:
            s[k] = 0

    def on_tick(self) -> None:
        if self._closed:
            return
        self._stats["tick"] += 1
        try:
            if self.sync_mode:
                self.world.tick()
        except RuntimeError as exc:
            self.get_logger().error(f"world.tick() 失败: {exc}")
            return

        if self.ego is None or not self.ego.is_alive:
            return

        # 执行器层：自己跑速度 PID，再 apply_control
        # 但如果最近没人发控制指令（比如换成 CARLA 官方 BehaviorAgent 在开），
        # 就放手别管，否则两边会互相打架。
        if (not self._autopilot) and (time.time() - self._last_cmd_time) < 0.5:
            self.ego.apply_control(self._speed_control(1.0 / self.rate_hz))

        tf = self.ego.get_transform()
        vel = self.ego.get_velocity()
        ang = self.ego.get_angular_velocity()

        stamp = self.get_clock().now().to_msg()
        px, py, pz = carla_location_to_ros(tf.location)
        qx, qy, qz, qw = carla_rotation_to_ros_quaternion(tf.rotation)
        vx, vy, vz = carla_vector_to_ros(vel)
        wx, wy, wz = carla_angular_velocity_to_ros(ang)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.position.z = pz
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        odom.twist.twist.angular.x = wx
        odom.twist.twist.angular.y = wy
        odom.twist.twist.angular.z = wz
        self.pub_odom.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = px
        t.transform.translation.y = py
        t.transform.translation.z = pz
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        if self.follow_spectator:
            spectator = self.world.get_spectator()
            spectator.set_transform(carla.Transform(
                tf.location + carla.Location(z=6.0, x=-8.0),
                carla.Rotation(pitch=-15.0, yaw=tf.rotation.yaw)))

    # -------------------------------------------------------------------- 收尾
    def destroy(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.get_logger().info("清理 CARLA actor ...")
        for actor in self.actors:
            try:
                if actor.is_alive:
                    actor.destroy()
            except Exception:  # noqa: BLE001
                pass
        self.actors.clear()
        if self._prev_settings is not None:
            try:
                settings = self.world.get_settings()
                settings.synchronous_mode = self._prev_settings[0]
                settings.fixed_delta_seconds = self._prev_settings[1]
                self.world.apply_settings(settings)
            except Exception:  # noqa: BLE001
                pass


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = CarlaBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        print(f"[carla_bridge] 启动失败: {exc}", file=sys.stderr)
        raise
    finally:
        if node is not None:
            node.destroy()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
