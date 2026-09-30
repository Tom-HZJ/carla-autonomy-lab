#!/usr/bin/env bash
# 进入工程环境：source setup/activate.sh
#
# 做四件事：
#   1. 激活 conda 里的 ROS 2 环境 (envs/ros2, Python 3.10)
#   2. 设好 ROS_DOMAIN_ID / RMW，避免和别的机器/进程抢 DDS
#   3. 把 CARLA 的 Python API 与发行包路径接上
#   4. 提供 ros2 / carla 的便捷别名

PROJ=/home/tom/Desktop/ROS2
CONDA_BIN=/home/tom/anaconda3/bin/conda

# 工程用 conda 里的 test 环境（Python 3.10）。
# 为什么必须是 3.10：carla 0.9.15 的 wheel 最高只到 cp310，
# 而 test 原本是 3.12，压根装不了 carla 客户端，所以把 test 重建成 3.10。
LAB_ENV="${LAB_ENV:-test}"

if [ ! -d "/home/tom/anaconda3/envs/$LAB_ENV" ]; then
  echo "!! conda 环境 $LAB_ENV 不存在，先跑 setup/create_ros_env.sh"
  return 1 2>/dev/null || exit 1
fi

# --- 1. 激活环境 ---
# 一定要先 source conda.sh，否则 conda activate 会报 "Run 'conda init' first"
# shellcheck disable=SC1091
if [ -f /home/tom/anaconda3/etc/profile.d/conda.sh ]; then
  source /home/tom/anaconda3/etc/profile.d/conda.sh
fi
conda activate "$LAB_ENV"

# --- 2. ROS 2 / DDS 设置 ---
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
# CycloneDDS 传大消息（720p 图像 ≈ 3.7 MB）比 Fast DDS 快得多：
# Fast DDS 实测一帧要 ~200 ms，CycloneDDS 只要几 ms。
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$PROJ/setup/cyclonedds.xml}"
export RCUTILS_COLORIZED_OUTPUT=1

# 顺手把本工程 colcon 出来的包 source 进来
if [ -f "$PROJ/ros2_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$PROJ/ros2_ws/install/setup.bash"

  # RoboStack 版 ROS 2 下，colcon 只生成了 pythonpath hook，
  # 不会自动把工作空间的 prefix 加进 AMENT_PREFIX_PATH，
  # 结果 ros2 pkg / ros2 run 找不到我们自己编译的包。这里手动补上。
  for _ws_prefix in "$PROJ"/ros2_ws/install/*/; do
    if [ -f "${_ws_prefix}share/ament_index/resource_index/packages/$(basename "$_ws_prefix")" ]; then
      case ":$AMENT_PREFIX_PATH:" in
        *":${_ws_prefix%/}:"*) ;;
        *) export AMENT_PREFIX_PATH="${_ws_prefix%/}${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}" ;;
      esac
    fi
  done
  unset _ws_prefix
fi

# --- 3. CARLA 相关路径 ---
export CARLA_HOME="$PROJ/carla/CARLA_0.9.15"
export CARLA_DOWNLOADS="$PROJ/carla/downloads"
# 注意：不要往 PYTHONPATH 里塞 CARLA 发行包自带的 egg/whl。
# 那些是 cp27/cp37 的，在 Python 3.10 下 import 会直接 segfault。
# 我们用 pip 装的 carla-0.9.15-cp310 wheel（见 setup/create_ros_env.sh 后的 pip install 步骤）。
if ! python -c "import carla" >/dev/null 2>&1; then
  echo "[warn] 当前环境里 import carla 失败，请执行："
  echo "       pip install carla==0.9.15 -i https://mirrors.aliyun.com/pypi/simple/"
fi

# --- 4. 便捷别名 ---
export ROS2_LAB="$PROJ"
alias carla-server='bash $PROJ/setup/run_carla.sh'
alias lab-status='cat $PROJ/STATUS.md'

echo "[carla-autonomy-lab] 环境已就绪"
echo "  env    : $LAB_ENV"
echo "  python : $(python -V 2>&1)"
echo "  ROS    : ${ROS_DISTRO:-<未设置>}"
echo "  CARLA  : $CARLA_HOME"
echo "  DOMAIN : $ROS_DOMAIN_ID"
