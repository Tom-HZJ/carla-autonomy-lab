#!/usr/bin/env bash
# 显卡 / 驱动 / Vulkan 自检
set -u

echo "== 1. 系统 =="
lsb_release -ds 2>/dev/null || cat /etc/os-release | head -1
uname -r

echo
echo "== 2. NVIDIA 驱动 =="
if command -v nvidia-smi >/dev/null; then
  nvidia-smi --query-gpu=name,driver_version,memory.total,memory.used --format=csv
else
  echo "!! 没找到 nvidia-smi"
fi

echo
echo "== 3. Vulkan / OpenGL 运行库 =="
ldconfig -p | grep -E 'libvulkan\.so|libGL\.so\.1|libEGL\.so\.1' || echo "!! 缺少图形运行库"

echo
echo "== 4. ICD 列表（Vulkan 用哪块盘） =="
ls /usr/share/vulkan/icd.d/ 2>/dev/null || echo "(无 /usr/share/vulkan/icd.d)"

echo
echo "== 5. 显存余量提醒 =="
echo "开发阶段目标：<= 1280x720 / Low 画质 / LiDAR 32 线，留 3GB 以上余量"

