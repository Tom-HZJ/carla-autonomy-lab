#!/usr/bin/env bash
# carla-autonomy-lab 总控脚本
#
#   bash setup/lab.sh start carla [headless|dev|epic]   启动 CARLA 服务端
#   bash setup/lab.sh start bridge [额外的 -p 参数...]   启动 CARLA<->ROS2 桥
#   bash setup/lab.sh start control [额外的 -p 参数...]  启动最小控制节点
#   bash setup/lab.sh start rviz                        启动 RViz2
#   bash setup/lab.sh stop                              停掉全部
#   bash setup/lab.sh status                            看谁活着
#   bash setup/lab.sh build                             colcon build
#   bash setup/lab.sh check m0|m3                       跑验收脚本

set -uo pipefail

PROJ=/home/tom/Desktop/ROS2
LOGS="$PROJ/logs"

mkdir -p "$LOGS"

start_detached() {   # start_detached <日志文件> <命令...>
  local log="$1"; shift
  setsid nohup "$@" > "$log" 2>&1 < /dev/null &
  disown
  echo "已启动 -> $log"
}

# conda / ROS 的 activate 脚本里引用了未定义的变量，
# 在 set -u 下会直接报 "CONDA_BUILD: unbound variable"，所以临时关掉。
activate_lab() {
  set +u
  # shellcheck disable=SC1091
  source "$PROJ/setup/activate.sh" >/dev/null
  set -u
}

stop_all() {
  echo "停 ROS 节点 ..."
  pkill -f "lib/carla_autonomy/carla_bridg[e]" 2>/dev/null
  pkill -f "lib/carla_autonomy/carla_contro[l]" 2>/dev/null
  pkill -f "lib/carla_autonomy/pure_pursui[t]" 2>/dev/null
  pkill -f "lib/carla_autonomy/mpc_controlle[r]" 2>/dev/null
  pkill -f "lib/carla_autonomy/obstacle_detecto[r]" 2>/dev/null
  pkill -f "lib/carla_autonomy/behavior_drive[r]" 2>/dev/null
  pkill -f "rviz[2]" 2>/dev/null
  sleep 2
  echo "停 CARLA 服务端 ..."
  pkill -f "CarlaUE4-Linux-Shippin[g]" 2>/dev/null
  # 一定要等端口真的释放，否则下一次 start 会连到"还没死透"的旧服务端，
  # 世界状态（残留车辆）就会串味。
  for _ in $(seq 1 40); do
    if ! ss -lnt 2>/dev/null | grep -q ':2000 '; then break; fi
    sleep 1
  done
  if ss -lnt 2>/dev/null | grep -q ':2000 '; then
    echo "!! 端口 2000 还占着，强杀"; pkill -9 -f "CarlaUE4-Linux-Shippin[g]" 2>/dev/null; sleep 3
  fi
  echo "已全部停止"
}

# 等 CARLA 起来能接受连接
wait_carla() {
  for _ in $(seq 1 90); do
    if ss -lnt 2>/dev/null | grep -q ':2000 '; then
      sleep 5
      echo "CARLA 已就绪"
      return 0
    fi
    sleep 1
  done
  echo "!! 等 CARLA 超时，看 logs/carla_server.log"
  return 1
}

status_all() {
  printf "%-22s %s\n" "CARLA 服务端" "$(pgrep -f 'CarlaUE4-Linux-Shippin[g]' >/dev/null && echo 运行中 || echo 未运行)"
  printf "%-22s %s\n" "carla_bridge" "$(pgrep -f 'lib/carla_autonomy/carla_bridg[e]' >/dev/null && echo 运行中 || echo 未运行)"
  printf "%-22s %s\n" "carla_control" "$(pgrep -f 'lib/carla_autonomy/carla_contro[l]' >/dev/null && echo 运行中 || echo 未运行)"
  printf "%-22s %s\n" "pure_pursuit" "$(pgrep -f 'lib/carla_autonomy/pure_pursui[t]' >/dev/null && echo 运行中 || echo 未运行)"
  printf "%-22s %s\n" "mpc_controller" "$(pgrep -f 'lib/carla_autonomy/mpc_controlle[r]' >/dev/null && echo 运行中 || echo 未运行)"
  printf "%-22s %s\n" "obstacle_detector" "$(pgrep -f 'lib/carla_autonomy/obstacle_detecto[r]' >/dev/null && echo 运行中 || echo 未运行)"
  printf "%-22s %s\n" "behavior_driver" "$(pgrep -f 'lib/carla_autonomy/behavior_drive[r]' >/dev/null && echo 运行中 || echo 未运行)"
  printf "%-22s %s\n" "rviz2" "$(pgrep -f 'rviz[2]' >/dev/null && echo 运行中 || echo 未运行)"
}

cmd="${1:-status}"
shift || true

case "$cmd" in
  start)
    what="${1:-}"; shift || true
    case "$what" in
      carla)   start_detached "$LOGS/carla_server.log" \
                   bash "$PROJ/setup/run_carla.sh" "${1:-headless}"
               wait_carla ;;
      bridge)  # shellcheck disable=SC1091
               activate_lab
               start_detached "$LOGS/bridge.log" ros2 run carla_autonomy carla_bridge --ros-args "$@" ;;
      control) # shellcheck disable=SC1091
               activate_lab
               start_detached "$LOGS/control.log" ros2 run carla_autonomy carla_control --ros-args "$@" ;;
      pursuit) # shellcheck disable=SC1091
               activate_lab
               start_detached "$LOGS/pursuit.log" ros2 run carla_autonomy pure_pursuit \
                 --ros-args -p route_file:="$PROJ/ros2_ws/src/carla_autonomy/config/route_town10.json" "$@" ;;
      mpc)     # shellcheck disable=SC1091
               activate_lab
               start_detached "$LOGS/mpc.log" ros2 run carla_autonomy mpc_controller \
                 --ros-args -p route_file:="$PROJ/ros2_ws/src/carla_autonomy/config/route_town10.json" "$@" ;;
      perception) # shellcheck disable=SC1091
               activate_lab
               start_detached "$LOGS/perception.log" ros2 run carla_autonomy obstacle_detector "$@" ;;
      agent)   # shellcheck disable=SC1091
               activate_lab
               start_detached "$LOGS/agent.log" ros2 run carla_autonomy behavior_driver "$@" ;;
      rviz)    # shellcheck disable=SC1091
               activate_lab
               start_detached "$LOGS/rviz.log" rviz2 -d "$PROJ/ros2_ws/src/carla_autonomy/config/ego.rviz" ;;
      *) echo "用法: bash setup/lab.sh start {carla|bridge|control|pursuit|mpc|perception|agent|rviz}"; exit 1 ;;
    esac
    ;;
  stop)   stop_all ;;
  status) status_all ;;
  build)  # shellcheck disable=SC1091
          activate_lab
          cd "$PROJ/ros2_ws" && colcon build --symlink-install ;;
  check)  # shellcheck disable=SC1091
          activate_lab
          case "${1:-m0}" in
            m0) python "$PROJ/scripts/check_m0.py" ;;
            m3) python "$PROJ/scripts/check_m3.py" ;;
            m5) python "$PROJ/scripts/check_m5.py" ;;
            *) echo "用法: bash setup/lab.sh check {m0|m3|m5}"; exit 1 ;;
          esac ;;
  *) echo "用法: bash setup/lab.sh {start|stop|status|build|check}"; exit 1 ;;
esac
