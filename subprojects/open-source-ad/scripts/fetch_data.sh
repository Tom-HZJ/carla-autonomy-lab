#!/usr/bin/env bash
# 下载 nuScenes v1.0-mini（4.17 GB，无需注册）
set -euo pipefail

PROJ=/home/tom/Desktop/ROS2
SUB="$PROJ/subprojects/open-source-ad"
URL=https://www.nuscenes.org/data/v1.0-mini.tgz
DEST="$SUB/data/v1.0-mini.tgz"

mkdir -p "$SUB/data"

# 复用主工程的多连接下载器：单连接会被限速，10 个连接能跑满带宽
bash "$PROJ/setup/par_download.sh" "$URL" "$DEST" 10 "$PROJ/logs/nuscenes_download.log"

echo "下载完成，开始解压 ..."
tar -xzf "$DEST" -C "$SUB/data"
echo "解压完成：$SUB/data/v1.0-mini"
ls "$SUB/data/v1.0-mini" | head

