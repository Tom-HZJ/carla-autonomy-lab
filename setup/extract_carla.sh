#!/usr/bin/env bash
# 解压 CARLA 0.9.15 发行包到 carla/CARLA_0.9.15/
set -euo pipefail

PROJ=/home/tom/Desktop/ROS2
TARBALL="$PROJ/carla/downloads/CARLA_0.9.15.tar.gz"
DEST="$PROJ/carla"

if [ ! -f "$TARBALL" ]; then
  echo "!! 找不到 $TARBALL，先跑 setup/download_carla.sh"
  exit 1
fi

mkdir -p "$PROJ/logs"

# 这个 tar 包没有顶层目录，直接是 CarlaUE4 / Engine / PythonAPI ...
# 所以先解到临时目录，再整体挪进 CARLA_0.9.15/
TARGET="$DEST/CARLA_0.9.15"
STAGE="$DEST/.extract_stage"

if [ -x "$TARGET/CarlaUE4.sh" ]; then
  echo "已经解压过了：$TARGET"
  du -sh "$TARGET"
  exit 0
fi

mkdir -p "$STAGE"
echo "解压 $TARBALL -> $STAGE （约需 5-15 分钟）"
tar -xzf "$TARBALL" -C "$STAGE" --checkpoint=2000 --checkpoint-action=dot
echo

if [ ! -x "$STAGE/CarlaUE4.sh" ] && [ -d "$STAGE/CARLA_0.9.15" ]; then
  STAGE="$STAGE/CARLA_0.9.15"
fi

mkdir -p "$TARGET"
( shopt -s dotglob; mv "$STAGE"/* "$TARGET"/ )
rmdir "$STAGE" 2>/dev/null

if [ -x "$TARGET/CarlaUE4.sh" ]; then
  echo "解压完成：$TARGET"
  du -sh "$TARGET"
  echo
  echo "提示：发行包自带的 Python API 只有 cp27/cp37 的 egg/whl，"
  echo "      我们用的是 pip 上的 carla-0.9.15-cp310 wheel，不需要这些 egg。"
else
  echo "!! 解压后没找到 CarlaUE4.sh，包内容："
  ls "$TARGET"
  exit 1
fi
