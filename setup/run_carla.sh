#!/usr/bin/env bash
# 启动 CARLA 服务端
#
# 用法:
#   bash setup/run_carla.sh            # 开发模式：Low 画质 + 720p + 有窗口
#   bash setup/run_carla.sh headless   # 无窗口（远程/纯计算）
#   bash setup/run_carla.sh epic       # 出片模式：Epic + 1080p

set -u

PROJ=/home/tom/Desktop/ROS2
CARLA_HOME="$PROJ/carla/CARLA_0.9.15"
MODE="${1:-dev}"

if [ ! -x "$CARLA_HOME/CarlaUE4.sh" ]; then
  echo "!! 还没解压 CARLA。先跑 setup/extract_carla.sh"
  exit 1
fi

case "$MODE" in
  headless)
    ARGS=(-RenderOffScreen -nosound -quality-level=Low -ResX=1280 -ResY=720)
    ;;
  epic)
    ARGS=(-nosound -quality-level=Epic -ResX=1920 -ResY=1080)
    ;;
  *)
    ARGS=(-nosound -quality-level=Low -windowed -ResX=1280 -ResY=720)
    ;;
esac

echo "启动 CARLA ($MODE): ${ARGS[*]}"
cd "$CARLA_HOME" || exit 1
exec ./CarlaUE4.sh "${ARGS[@]}"

