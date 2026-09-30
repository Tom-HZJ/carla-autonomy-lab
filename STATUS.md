# 项目进度看板

最后更新：2026-09-30

## 恢复工作（下次开工先看这里）

仿真已于 2026-09-30 中午关闭，现场干净（无残留进程，GPU 已释放，端口 2000 已释放）。
下午继续开发时，从这里接：

```bash
cd /home/tom/Desktop/ROS2

# 1. 起仿真 + 控制器 + 可视化（约 30 秒）
bash setup/lab.sh start carla headless
bash setup/lab.sh start bridge
bash setup/lab.sh start pursuit        # M5 纯跟踪（稳）
bash setup/lab.sh start rviz

# 2. 验收 / 观察
bash setup/lab.sh check m5             # 90 秒跑一圈，看横向误差

# 3. 结束
bash setup/lab.sh stop
```

想自己开 CARLA 的图形窗口看雨夜场景：`bash setup/lab.sh start carla dev`
（会弹窗，用 `-windowed -quality-level=Low`）。

本次接续点：**做 M7 感知**（LiDAR 聚类避障 + 接 YOLO 检测），然后是 M6 收尾、M8。
详见文末「下一步」。

备注：sudo 密码存在 `/home/tom/.lab_sudo_pw`（权限 600）。
后续 M7/M8 基本不需要 sudo，要删随时说一声。

## 当前阶段

M0 – M5 已完成并且稳定。M6（MPC）已实现，直线和小曲率段跟踪比 M5 更好，
但还有一个偶发的跑丢点没根因定位，**默认转向仍用 M5 的纯跟踪**。

## 步骤清单

| # | 步骤 | 状态 | 说明 |
|---|---|---|---|
| 0.1 | 需求来源归档（DeepSeek 对话） | 完成 | docs/deepseek_conversation.txt |
| 0.2 | 本机勘察（GPU/内存/磁盘/sudo/网络） | 完成 | 见 PLAN.md 第 2 节 |
| 0.3 | 方案定稿（含 3 个关键技术决策） | 完成 | PLAN.md |
| 0.4 | 下载 CARLA 0.9.15 发行包（8.4 GB） | 完成 | 8386636048 字节，与服务器 content-length 完全一致 |
| 0.5 | 创建 ROS 2 Humble conda 环境（py3.10） | 完成 | envs/ros2，Python 3.10.13，275 个 ROS 包 |
| 0.6 | 解压 CARLA 到 carla/CARLA_0.9.15/ | 进行中 | logs/extract_carla.log |
| 0.7 | 装 carla==0.9.15 wheel 进 ROS 2 环境 | 完成 | import carla 通过 |
| 0.8 | M0 验收：CARLA 服务端 + 客户端连通 | 待办 | scripts/check_m0.py |

### 里程碑

| 阶段 | 内容 | 状态 | 验收结果 |
|---|---|---|---|
| M0 | CARLA 服务端 + 客户端连通 | 完成 | client/server 0.9.15，Town10HD_Opt，155 spawn 点 |
| M1 | Town10HD + 雨夜渲染 | 完成 | datasets/camera/rgb_010495.png（湿滑路面 + 雨） |
| M2 | 自车 + Camera/LiDAR/IMU/GNSS | 完成 | 2 秒采到 61 帧图像 / 131 帧点云 / IMU 130 条 |
| M3 | 传感器进 ROS 2 + 控制回流 | 完成 | 脚本 scripts/check_m3.py 全绿，车自己跑了 35.42 m |
| M4 | RViz2 可视化 | 完成 | rviz2 OpenGL 4.6 起来，点云/相机/里程计/路径/TF 全在 config/ego.rviz |
| M5 | Pure Pursuit + PID | 完成 | 90s 跑 685 m（2.5 圈），横向误差均值 0.291 m / 95% 0.495 m / 最大 0.929 m |
| M6 | MPC (CasADi) | 部分完成 | 能跑，直线段 XTE 0.1–0.4 m；偶发一次跑丢（约 634 m 后），未根因定位 |
| M7 | 感知 + 避障 | 基本完成 | 两条腿：①自研 LiDAR 体素+并查集聚类检测（1–2ms，出包围盒）；②复用 CARLA 官方 agents 驾驶，409 m 自主到终点、0 次急停 |
| M8 | ST-Graph + MPC | 待办 | |

M3 实测数据：

```
camera :  1280x720 bgra8   （桥以 20Hz 发布，Python 订阅者本身跟不上）
lidar  :  19.6 Hz, 16633 点/帧, 32 线
imu    :  19.6 Hz
gnss   :  19.6 Hz
odom   :  20.0 Hz
位移   :  35.42 m（靠我们发的 Twist 驱动）
```

M5 实测数据（`bash setup/lab.sh check m5`）：

```
行驶距离 :  685.5 m  (平均 7.6 m/s)
横向误差 :  均值 0.291 m / 95% 0.495 m / 最大 0.929 m
路线     :  273 点，272 m，接缝 1.84 m，闭环
```

出片效果：`datasets/camera/demo_00..03.png`（雨夜路口，湿滑路面反光）

## 环境备忘

- 工作目录：/home/tom/Desktop/ROS2
- ROS 2 环境前缀：envs/ros2（conda，Python 3.10）
- 进入环境：source setup/activate.sh
- 日志目录：logs/

## 关键结论（避免下次重复踩坑）

- 无 sudo，故 ROS 2 走 RoboStack conda，不用 apt。
- 工程锁 Python 3.10：carla 0.9.15 wheel 最高到 cp310，RoboStack humble 有 py310 构建。
- CARLA 官方下载地址是 downloads.carlasim.com（不是 GitHub，GitHub 在这台机器上不可达）。
- 只有 IPv4 能通 CARLA CDN，必须 curl -4。
- CARLA CDN 单连接会被限速到几十 KB/s，用 setup/par_download.sh 并发 10 连接可到 ~36 MB/s。
- 后台任务要用 setsid 启动，否则 exec 会话结束会连子进程一起杀掉。
- CARLA 发行包自带的 PythonAPI 只有 cp27/cp37 的 egg/whl，
  把它们放进 PYTHONPATH 会在 py3.10 下直接 segfault —— 必须用 pip 的 cp310 wheel。
- CARLA 0.9.15 没有 `Actor.get_autopilot()`，自动驾驶状态要自己记。
- CARLA 0.9.15 没有 `Rotation.get_quaternion()`，
  姿态要走 `get_forward_vector/right/up` 拼旋转矩阵（见 carla_autonomy/transforms.py）。
- **最大的性能坑**：`msg.data = bytes(...)` 会走 rosidl 生成代码里的 `__debug__` 校验分支，
  对每个元素跑两次 Python 级 `all(...)`，3.7MB 的图像要 ~200ms！
  必须改成 `msg.data = array.array('B', raw)`，直接命中快路径，降到 ~1ms。
- RoboStack 下 colcon 只生成 pythonpath hook，不会自动把工作空间加进 AMENT_PREFIX_PATH，
  activate.sh 里已手动补上。
- 本机 RMW 用 rmw_cyclonedds_cpp（见 setup/cyclonedds.xml）。
- **转向符号**：CARLA `VehicleControl.steer` 正值 = 右转，ROS 是左转为正，
  两者相反。忘了取反的表现是车一进弯就正反馈失控画圈。
  已实测（steer=+0.4 → CARLA yaw +74.7°，等价 ROS yaw -74.7°）。
- **闭环接缝**：沿车道爬出来的路线，末点离起点可能差 8 m，
  直接当闭环用，纯跟踪在接缝处会看到突跳把车甩出去。
  现在 route.close_loop() 会把尾巴剪到离起点最近的点，再接上补点，接缝降到 ~2 m。
- **纯跟踪会切内道**：稳态横向偏差 ≈ Ld²/8R。
  加一个 Stanley 式横向误差反馈 `delta -= atan(k·e/v)` 后，
  均值从 0.43 m 降到 0.29 m。
- **纵向 PID 放在桥里**（执行器层），上层控制器只发运动学指令。
- **lab.sh stop 必须等端口 2000 真的释放**，否则下一次 start 会连到没死透的旧服务端，
  残留车辆会串味（这个坑排查了很久）。
- 路径要周期性重发，RViz 后连上来才看得到（VOLATILE 只发一次会丢）。
- **转向增益标定**（很重要，M5/M6 都受影响）：
  CARLA `VehicleControl.steer=1.0` ≈ 前轮 **1.0 rad**，不是想当然的 0.7。
  实测 steer=0.10→5.69°、0.20→8.86°、0.45→24.40°。
  桥里 `max_steer_rad` 填错的话，上层算出的转角会被放大 1/0.7≈1.43 倍执行，
  表现是"控制器以为打 12°、车实际打 17°"，MPC 的模型和真车就对不上。
  改成 1.0 之后 M5 的均值从 0.291 → 0.269 m。
- **MPC 的两个坑**：
  1. 不能把"未来 1 步的速度"当速度目标发给下层（加速度受限，1 步只抬 0.25 m/s，
     车就爬不动），要取前瞻若干步的速度当目标；
  2. 转角约束要分两个值：限速用保守的 a_lat（舒适），
     转角上限用宽松的 a_lat_hard（出误差时得打得回来）。
- 有 sudo 可用，密码文件 `/home/tom/.lab_sudo_pw`（600）。已装 tmux/aria2/vulkan-tools/nvtop。

## 常用命令

```bash
bash setup/lab.sh start carla headless   # 起 CARLA 服务端
bash setup/lab.sh start bridge           # 起 CARLA<->ROS2 桥
bash setup/lab.sh start pursuit          # 起纯跟踪控制器
bash setup/lab.sh start mpc              # 起 MPC 控制器（实验性）
bash setup/lab.sh start rviz             # 起 RViz2
bash setup/lab.sh status
bash setup/lab.sh check m3|m5
bash setup/lab.sh stop
```

生成路线（换起点就换 spawn_index）：
```bash
ros2 run carla_autonomy make_route --ros-args -p spawn_index:=0
```

## 下一步（下次开工从这里继续）

1. **M7（优先级最高）**：目标检测 + 障碍物处理。
   - LiDAR 点云聚类（欧式聚类）→ 几何障碍物 → 减速/停车/绕行；
   - 相机 YOLO 检测（本机已有 `ultralytics-8.3.208` + `yolov8m.pt/onnx/engine`），
     发布 `/carla/ego/detections`。
2. **M6 收尾**：MPC 偶发跑丢的问题。怀疑和弯中侧偏（understeer）与转角约束边界有关，
   下一步打算把 a_lat_hard 再放宽、并把 horizon 拉长到 2.5s 试。
3. **M8**：ST-Graph 时空联合规划。
4. 录像：把相机帧序列拼成 mp4，出片用。

## M7 阶段小结（2026-09-30 下午）

### M7c YOLO 目标检测（2026-09-30 傍晚）

- **换用官方通用权重**：`models/yolov8n.pt`（6.5 MB）/ `yolov8s.pt`（22.6 MB），
  80 类 COCO。用户本地那份 `ultralytics-8.3.208/data/yolov8m.pt` 是微调过的，
  只认很少几类（`CLASS_MAP` 里只有 person），所以不用它。
- **检测节点** `yolo_detector_node.py`：
  `/carla/ego/camera/image_raw` → YOLO → `/carla/ego/detections`（JSON）
  + `/carla/ego/detections_image`（画框图）+ `/carla/ego/detection_count`。
  实测 GPU 推理 **6–10 ms/帧**，雨夜场景能认出红绿灯和行人。
- 默认 `conf=0.35`；夜里小目标要靠 `-p conf:=0.15` 才出得来。

### 环境合并（这一步踩的坑最多）

`yolo` 环境是 Python 3.10 + torch 2.8.0+cu128 + ultralytics 8.3.213，CUDA 可用；
我们的 ROS 环境也是 Python 3.10，所以直接把 GPU 依赖装进 `envs/ros2`，
一个进程里既有 rclpy 又有 YOLO，省掉跨进程通信。

但 pip 装 ultralytics 会顺手把 **numpy 顶到 2.x**，而 RoboStack 里一堆包是按
numpy 1.x 的 ABI 编的，结果 `cv2` 直接 import 失败（`_ARRAY_API not found`）。
解决办法是 `setup/pip-constraints.txt` 把 `numpy==1.26.4` 和 `opencv-python==4.6.0`
钉死，之后所有 pip 安装都带 `-c`。

另外两个坑：
- torch 从 PyPI 默认装到 **2.14+cu130**，cuDNN 加载失败
  （`CUDNN_STATUS_SUBLIBRARY_LOADING_FAILED`）。降到 **2.8.0+cu128** 正常。
- pip 把 **setuptools 升到 84**，`setup.py develop --uninstall` 被移除，
  colcon 直接构建失败。降到 `setuptools<70` 恢复。

### 常用命令补充

```bash
bash setup/lab.sh start yolo              # 起 YOLO 检测（默认 conf 0.35）
bash setup/lab.sh start yolo -p conf:=0.15 -p every_n:=3   # 夜里小目标
```

### 做成了什么

- **自研 LiDAR 障碍物检测**（`obstacle_detector.py`）：体素 + 并查集聚类，
  1–2 ms，出包围盒 / 最近距离，RViz 可看，话题 `/carla/ego/obstacles`。
- **复用 CARLA 官方 agents 包**（`behavior_driver.py`）：
  `BehaviorAgent` + `GlobalRoutePlanner` + `LocalPlanner`，
  实测 **409 m 自主行驶到指定终点，全程 7.5 m/s，0 次急停**，
  路上散布 14 个路障也没撞。
- 桥新增两个健壮性：控制指令超时 0.5 s 就放手（让官方 agent 接管，
  两边不打架）；spawn 点被占自动换一个。

### 为什么改成"复用官方 agents"

自己手写的绕行避障过不去正中央的路障：绕行偏移加在"前瞻点"上，
障碍物进到 3–4 m 才反应得过来，横向来不及挪。
CARLA 官方 `BasicAgent` 的避障用的是**仿真真值 actor 列表 + 路线多边形**，
比用 LiDAR 去猜稳得多，而且自带红绿灯和路口让行。

所以 M7 改成两条腿：官方 agent 负责"开得好"，
自研 LiDAR 聚类负责"感知演示 / 学习对照"。两者都保留。

### 这一阶段新踩的坑

- **LiDAR 会打到自己的引擎盖**：车顶 2.5 m 的 LiDAR 斜向下能看到自己车头，
  稳定报"前方 2.4 m 有障碍"，车就一直对着自己刹车。`roi_x_min` 要 ≥ 3 m。
- **`.gitignore` 行尾不能写注释**：`envs/   # 环境` 会被当成完整模式，
  结果大目录没被忽略，`git add` 卡死还往 `.git` 塞了 6 GB 垃圾。
- **新连上来的 CARLA 客户端第一次 `get_actors()` 常常返回空**，
  所有"第二客户端"查询都要带重试。
- **同步模式下第二个客户端做 spawn/查询容易超时**，
  所以摆障碍物要在起桥之前做。
