#!/usr/bin/env bash
# 复现 ROS 2 Humble conda 环境（Python 3.10，无需 root）
set -euo pipefail

PROJ=/home/tom/Desktop/ROS2
PREFIX="$PROJ/envs/ros2"

mkdir -p "$PROJ/logs"

conda create -y -p "$PREFIX" \
  -c robostack-staging -c conda-forge --strict-channel-priority \
  python=3.10 \
  ros-humble-desktop \
  ros-humble-rosbag2 \
  colcon-common-extensions

echo
echo "ROS 2 环境建好了：$PREFIX"
echo "用 source setup/activate.sh 进入"

