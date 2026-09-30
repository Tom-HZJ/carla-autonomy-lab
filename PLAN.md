# CARLA Autonomy Lab —— 项目方案与开发计划

> 目标：在本机（无 root 权限）从零搭出一套「CARLA 自动驾驶闭环仿真」工程，
> 手写定位 → 规划 → 控制算法，最终在 Town10HD 雨夜场景里跑通 M0–M8。

最后更新：2026-09-30

## 1. 项目目标

跑通这条闭环，并且每一环都是我们自己能看懂、能改的代码：

```
CARLA 0.9.15 (Town10HD / 雨夜)
   → Camera / LiDAR / IMU / GNSS
   → ROS 2 Humble 话题
   → 定位 (里程计 + 地图坐标)
   → 全局路径 (车道中心线 / waypoints)
   → 局部规划 (避障, ST-Graph)
   → 控制 (Pure Pursuit + PID → MPC)
   → 回到 CARLA 车辆 (Ackermann 控制指令)
```

刻意不使用 Autoware 等全家桶，避免“调得动但看不懂”。

## 2. 本机实际情况（已实测）

| 项 | 实测值 | 备注 |
|---|---|---|
| 系统 | Ubuntu 22.04.5 LTS，内核 6.8.0-85 | 满足要求 |
| CPU | 32 线程 | 足够 |
| 内存 | 31 GiB | 满足要求 |
| GPU | RTX 4080 Laptop，12282 MiB (12 GB)，驱动 580.65.06 | 方案里写的 5080/16GB 有误，按 12GB 定策略 |
| 磁盘 | /home 剩余 260 GB | 够（CARLA + envs 约 20 GB） |
| 图形栈 | libvulkan.so.1 / libGL.so.1 / libEGL.so.1 均存在 | CARLA 需要 Vulkan |
| sudo | 需要密码，不可用 | 全部依赖改为“装在工程目录里” |
| ROS 2 | 未安装 | 改走 conda (RoboStack) |
| 网络 | github.com 不可达；api.github.com / conda / PyPI 镜像可用 | CARLA 改从官方 CDN 下载 |

### 2.1 因为 sudo 不可用而做的三个关键决策

决策 A：ROS 2 Humble 用 RoboStack（conda）而不是 apt。

```
conda create -p ./envs/ros2 -c robostack-staging -c conda-forge python=3.10 ros-humble-desktop
```

RoboStack 提供 ROS 2 Humble 的原生 conda 构建（含 rviz2 / rclpy / tf2 / rosbag2），
完全不需要 root，且能装进工程目录。

决策 B：整个工程锁 Python 3.10。这是唯一同时满足下面两条的版本：

- carla==0.9.15 在 PyPI 上只提供 cp37/cp38/cp39/cp310 的 wheel（没有 cp311/cp312）；
- RoboStack 的 ros-humble-ros-base 有 py310h7c61026 构建。

顺带纠正原方案里的一个误判：它担心“CARLA 只有 py3.7/3.8 的 egg”，
实际上 0.9.15 官方就发布了 cp310 wheel，且 0.9.15 的 changelog 明确写了
“Fixed segfaults in Python API due to incorrect GIL locking under Python 3.10”，
说明 py3.10 是官方支持的。不需要 conda 建 py3.8 环境，也不需要手动挂 .egg。

决策 C：CARLA ↔ ROS 2 的桥我们自己写，不编译官方 ros-bridge。

官方 carla-ros-bridge（0.9.15 版）要 colcon 编译十几个包，依赖
ackermann_msgs / derived_object_msgs / carla_msgs 等，在无 root 的
RoboStack 环境里依赖链风险很高。
我们改为在 ROS 2 环境里直接 import carla（cp310 wheel），写一个轻量桥接节点：
CARLA 传感器 → sensor_msgs/{Image,PointCloud2,Imu,NavSatFix}，
控制侧订阅自建的 AckermannDriveStamped 消息。约 300 行，完全可控，
也符合“手写”的路线。

### 2.2 12 GB 显存的画质策略

CARLA 雨夜 + Town10HD 在 12GB 显存下可行，但必须分阶段：

- 开发/调算法：-quality-level=Low -ResX=1280 -ResY=720，camera 720p，LiDAR 32 线 20 Hz。
- 出片/录视频：-quality-level=Epic -ResX=1920 -ResY=1080，LiDAR 64 线 10 Hz。
- 跑之前用 nvidia-smi 留够余量，避免 OOM 把 UE4 渲染线程打死。

## 3. 目录结构

```
/home/tom/Desktop/ROS2/
├── PLAN.md                 # 本文件：方案
├── STATUS.md               # 进度看板：每完成一步就更新
├── docs/                   # 需求来源（DeepSeek 对话存档）
├── setup/                  # 环境安装/启动脚本
│   ├── activate.sh         # 进入工程环境（source 它）
│   ├── create_ros_env.sh   # 复现 ROS 2 conda 环境
│   ├── download_carla.sh   # 下载 CARLA 发行包
│   ├── run_carla.sh        # 启动 CARLA 服务端
│   └── check_gpu.sh        # 显卡/驱动/Vulkan 自检
├── envs/ros2/              # ROS 2 Humble + Python 3.10 (conda)
├── carla/
│   ├── downloads/          # 下载缓存（tar.gz）
│   └── CARLA_0.9.15/       # 解压后的发行包
├── ros2_ws/src/carla_autonomy/   # 我们的 ROS 2 包
├── scripts/                # 独立的 CARLA 调试脚本
├── datasets/{camera,lidar,rosbag}   # 采集数据
├── models/                 # 后续 YOLO 等模型
└── logs/                   # 安装日志、运行日志
```

## 4. 里程碑与验收标准

| 阶段 | 内容 | 验收标准（必须可复现） | 状态 |
|---|---|---|---|
| M0 | CARLA 服务端跑起来 | CarlaUE4.sh 能起，Python 客户端能 get_world() 并打印地图名 | 待办 |
| M1 | Town10HD + 雨夜 | 加载 Town10HD_Opt，设成夜晚 + 大雨，能出图证明 | 待办 |
| M2 | 自车 + 传感器 | 生成 ego vehicle，挂 Camera/LiDAR/IMU/GNSS，数据能存盘 | 待办 |
| M3 | 传感器进 ROS 2 | 自研桥接节点把 4 类传感器发布成 ROS 2 话题，ros2 topic hz 有数据 | 待办 |
| M4 | RViz2 可视化 | 点云 + 车辆位姿 + 轨迹同屏，TF 树不断 | 待办 |
| M5 | Pure Pursuit + PID | 车能沿 waypoints 跑完一圈，横向误差 < 0.5 m | 待办 |
| M6 | MPC | CasADi MPC 接管控制，跟踪误差优于 M5 | 待办 |
| M7 | LiDAR 动态避障 | 前方障碍物触发减速/换道，不撞 | 待办 |
| M8 | ST-Graph + MPC | 动态障碍下的时空联合规划，复刻原帖效果 | 待办 |

## 5. 风险与对策

| 风险 | 可能性 | 对策 |
|---|---|---|
| CARLA 8.4 GB 下载中断/变慢 | 中 | curl -C - 断点续传 + --retry-all-errors，可随时重跑 |
| RoboStack 包依赖冲突 | 中 | 锁 python=3.10 + strict channel priority；失败则退回 ros-humble-ros-base 逐包加 |
| rviz2 在 4080 + 580 驱动下 OpenGL 异常 | 中 | 备选：改用 Foxglove（浏览器可视化）或用 CARLA 自带 spectator 出图 |
| UE4 在新驱动下渲染黑屏 | 中 | 试 -vulkan / -opengl；仍失败则 -quality-level=Low |
| CARLA server 与 ROS 2 抢 CPU | 低 | tmux 分屏 + ROS_DOMAIN_ID 隔离 + 限制 CARLA 帧率 |
| 显存不足（12 GB） | 中 | 严格按 2.2 节画质策略，先 Low 后 Epic |

## 6. 工作方式（长期迭代约定）

1. 每一步都写进 STATUS.md：做到哪、卡在哪、下一步干什么。中断后读 STATUS.md 就能续上。
2. 能自动验证的就自动验证：每个里程碑都有脚本化验收命令，不靠“看起来对”。
3. 先跑通最小闭环，再加难度：M5 之前不碰 MPC，M7 之前不碰 ST-Graph。
4. 所有新增依赖都装在工程目录内，不污染系统。

