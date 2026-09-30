#!/usr/bin/env bash
# 复现工程用的 conda 环境（不需要 root）
#
# 工程直接复用 conda 里的 **test** 环境，不再自建 envs/ros2。
# 注意：必须重建为 Python 3.10 —— 原 test 是 3.12，
# 而 carla 0.9.15 的 wheel 最高只到 cp310，装不上客户端。
#
# 用法:
#   bash setup/create_ros_env.sh
#   LAB_ENV=别的名字 bash setup/create_ros_env.sh
set -euo pipefail

PROJ=/home/tom/Desktop/ROS2
LAB_ENV="${LAB_ENV:-test}"
PY_VER="${PY_VER:-3.10}"
PY="/home/tom/anaconda3/envs/$LAB_ENV/bin/python"

mkdir -p "$PROJ/logs"

echo "== 1. 删掉旧的 $LAB_ENV（如果有） =="
conda env remove -n "$LAB_ENV" -y 2>/dev/null || true

echo "== 2. 建 $LAB_ENV: Python $PY_VER + ROS 2 Humble =="
# --override-channels: 只用可达的 robostack / conda-forge，
# 免得 conda 去连 repo.anaconda.com 干等超时。
conda create -y -n "$LAB_ENV" --override-channels \
  -c robostack-staging -c conda-forge --strict-channel-priority \
  "python=$PY_VER" \
  ros-humble-desktop \
  ros-humble-rosbag2 \
  colcon-common-extensions

echo "== 3. 装 Python 侧依赖（carla / MPC / YOLO） =="
# 一定要带 -c 约束：pip 装 ultralytics 会把 numpy 顶到 2.x，
# 而 RoboStack 里 cv2 等是按 numpy 1.x 的 ABI 编译的，一升全挂。
"$PY" -m pip install -c "$PROJ/setup/pip-constraints.txt" \
  -i https://pypi.tuna.tsinghua.edu.cn/simple \
  "carla==0.9.15" pillow casadi \
  "torch==2.8.0" "torchvision==0.23.0" \
  scipy pandas tqdm py-cpuinfo ultralytics ultralytics-thop

echo
echo "环境建好了：conda 的 $LAB_ENV"
echo "用 source setup/activate.sh 进入"

