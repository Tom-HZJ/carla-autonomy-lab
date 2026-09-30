#!/usr/bin/env python3
"""M7c: YOLOv8 目标检测节点（相机路）。

跟 obstacle_detector（LiDAR 几何聚类）互补：
    - LiDAR 聚类：知道"那儿有个东西、多远、多大"，但不知道是什么；
    - YOLO：知道"那是人/车/红绿灯"，但在图像上，没有精确距离。
    两个一起用才是完整感知。

输入:  /carla/ego/camera/image_raw        (bgra8)
输出:  /carla/ego/detections_image        (bgr8, 画好框的图，给 RViz / 出片)
      /carla/ego/detections               (std_msgs/String, JSON 列表)
      /carla/ego/detection_count          (std_msgs/Int32)

每帧 JSON 的格式（归一化坐标，0~1，方便上层直接用）:
    [{"cls": 0, "name": "person", "conf": 0.83,
      "x": 0.51, "y": 0.62, "w": 0.07, "h": 0.21}, ...]

用法:
    ros2 run carla_autonomy yolo_detector --ros-args \
        -p model:=/home/tom/Desktop/ROS2/models/yolov8n.pt -p every_n:=2
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String


SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1,
                        durability=DurabilityPolicy.VOLATILE)


class YoloDetector(Node):
    def __init__(self) -> None:
        super().__init__("yolo_detector")

        default_model = str(Path("/home/tom/Desktop/ROS2/models/yolov8n.pt"))
        self.declare_parameter("model", default_model)
        self.declare_parameter("conf", 0.35)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", 0)          # 0 = 第一块 GPU
        self.declare_parameter("every_n", 2)         # 每 N 帧跑一次，省算力
        self.declare_parameter("classes", "")        # 逗号分隔的类别 id，空=全部
        self.declare_parameter("publish_image", True)

        self.model_path = str(self.get_parameter("model").value)
        self.conf = float(self.get_parameter("conf").value)
        self.iou = float(self.get_parameter("iou").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = self.get_parameter("device").value
        self.every_n = max(1, int(self.get_parameter("every_n").value))
        self.publish_image = bool(self.get_parameter("publish_image").value)
        cls_raw = str(self.get_parameter("classes").value).strip()
        self.classes = [int(c) for c in cls_raw.split(",") if c.strip()] or None

        # torch / ultralytics 导入很重，放到构造里一次，别放回调
        self.get_logger().info(f"加载 YOLO 模型 {self.model_path} ...")
        from ultralytics import YOLO
        self.model = YOLO(self.model_path)
        self.names = self.model.names
        self.get_logger().info(
            f"模型就绪，共 {len(self.names)} 类；conf={self.conf} imgsz={self.imgsz} "
            f"device={self.device}")

        self._frame_i = 0
        self._ms = 0.0
        self._last_log = 0.0

        self.create_subscription(Image, "/carla/ego/camera/image_raw",
                                 self.on_image, SENSOR_QOS)
        self.pub_img = self.create_publisher(Image, "/carla/ego/detections_image", 10)
        self.pub_json = self.create_publisher(String, "/carla/ego/detections", 10)
        self.pub_count = self.create_publisher(Int32, "/carla/ego/detection_count", 10)

    # ------------------------------------------------------------------
    def on_image(self, msg: Image) -> None:
        self._frame_i += 1
        if self._frame_i % self.every_n != 0:
            return

        t0 = time.perf_counter()
        frame = self._to_bgr(msg)
        if frame is None:
            return

        try:
            results = self.model.predict(frame, conf=self.conf, iou=self.iou,
                                         imgsz=self.imgsz, device=self.device,
                                         classes=self.classes, verbose=False)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"推理失败: {exc}", throttle_duration_sec=5.0)
            return
        self._ms = (time.perf_counter() - t0) * 1000.0

        r = results[0]
        h, w = frame.shape[:2]
        dets = []
        if r.boxes is not None and len(r.boxes) > 0:
            xyxy = r.boxes.xyxy.cpu().numpy()
            confs = r.boxes.conf.cpu().numpy()
            clss = r.boxes.cls.cpu().numpy().astype(int)
            for (x1, y1, x2, y2), cf, cid in zip(xyxy, confs, clss):
                dets.append({
                    "cls": int(cid),
                    "name": str(self.names.get(int(cid), cid)),
                    "conf": round(float(cf), 3),
                    "x": round(float((x1 + x2) / 2 / w), 4),
                    "y": round(float((y1 + y2) / 2 / h), 4),
                    "w": round(float((x2 - x1) / w), 4),
                    "h": round(float((y2 - y1) / h), 4),
                })

        self.pub_json.publish(String(data=json.dumps(dets, ensure_ascii=False)))
        self.pub_count.publish(Int32(data=len(dets)))

        if self.publish_image:
            self.pub_img.publish(self._annotated(msg, r, frame))

        now = time.time()
        if now - self._last_log > 3.0:
            self._last_log = now
            top = ", ".join(f"{d['name']}x{sum(1 for e in dets if e['name'] == d['name'])}"
                            for d in dets[:4]) or "无"
            self.get_logger().info(f"检出 {len(dets)} 个目标 [{top}]  {self._ms:.1f} ms/帧")

    # ------------------------------------------------------------------
    @staticmethod
    def _to_bgr(msg: Image) -> np.ndarray | None:
        try:
            if msg.encoding == "bgra8":
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
                return np.ascontiguousarray(arr[:, :, :3])
            if msg.encoding in ("rgb8", "bgr8"):
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                bgr = arr[:, :, ::-1] if msg.encoding == "rgb8" else arr
                return np.ascontiguousarray(bgr)
        except Exception:  # noqa: BLE001
            return None
        return None

    def _annotated(self, src: Image, r, frame) -> Image:
        """把画好框的图塞回 sensor_msgs/Image（bgr8）。"""
        plotted = r.plot()                      # ultralytics 自带的画框
        out = Image()
        out.header = src.header
        out.height, out.width = plotted.shape[:2]
        out.encoding = "bgr8"
        out.is_bigendian = False
        out.step = out.width * 3
        # 和桥里一样的坑：直接给 bytes 会走 rosidl 的慢校验分支，用 array 走快路径
        from array import array
        out.data = array("B", plotted.tobytes())
        return out


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = YoloDetector()
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

