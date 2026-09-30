#!/usr/bin/env bash
# 下载 CARLA 0.9.15 发行包（断点续传，只用 IPv4）
set -euo pipefail

PROJ=/home/tom/Desktop/ROS2
URL=https://downloads.carlasim.com/Linux/CARLA_0.9.15.tar.gz
DEST="$PROJ/carla/downloads/CARLA_0.9.15.tar.gz"

mkdir -p "$(dirname "$DEST")" "$PROJ/logs"

echo "下载 $URL"
echo "目标 $DEST"
curl -4 -L -C - --retry 8 --retry-delay 5 --retry-all-errors \
  -o "$DEST" "$URL"

echo "完成：$(du -h "$DEST" | cut -f1)"

