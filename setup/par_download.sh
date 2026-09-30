#!/usr/bin/env bash
# 多连接并行下载（CDN 单连接限速很凶，靠并发吃满带宽）
# 支持断点续传：每个分片单独校验大小，已完成的跳过。
#
# 用法: bash setup/par_download.sh <url> <输出文件> [并发数] [日志]

set -uo pipefail

URL="${1:?需要 URL}"
OUT="${2:?需要输出文件}"
PARTS="${3:-8}"
LOG="${4:-/home/tom/Desktop/ROS2/logs/par_download.log}"

mkdir -p "$(dirname "$OUT")" "$(dirname "$LOG")" "$(dirname "$OUT")/.parts"
PARTS_DIR="$(dirname "$OUT")/.parts"
BASE="$(basename "$OUT")"

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG"; }

# --- 1. 拿总长度 ---
TOTAL=$(curl -4 -sIL "$URL" | awk 'BEGIN{IGNORECASE=1} /^content-length:/ {gsub("\r","");print $2}' | tail -1)
if [ -z "$TOTAL" ]; then
  log "!! 拿不到 content-length，退回单连接下载"
  curl -4 -L -C - --retry 20 --retry-delay 5 --retry-all-errors -o "$OUT" "$URL"
  exit $?
fi

CHUNK=$(( (TOTAL + PARTS - 1) / PARTS ))
log "总大小 $TOTAL 字节，切成 $PARTS 片，每片约 $((CHUNK/1048576)) MiB"

# --- 2. 并行下载分片 ---
pids=()
for ((i=0; i<PARTS; i++)); do
  START=$(( i * CHUNK ))
  [ "$START" -ge "$TOTAL" ] && break
  END=$(( START + CHUNK - 1 ))
  [ "$END" -ge "$TOTAL" ] && END=$(( TOTAL - 1 ))
  EXPECT=$(( END - START + 1 ))
  PART="$PARTS_DIR/${BASE}.part$(printf '%03d' "$i")"

  if [ -f "$PART" ] && [ "$(stat -c %s "$PART")" -eq "$EXPECT" ]; then
    log "分片 $i 已完整，跳过"
    continue
  fi

  (
    for attempt in 1 2 3 4 5 6 7 8 9 10; do
      curl -4 -sL --retry 5 --retry-delay 3 --retry-all-errors \
           --speed-time 60 --speed-limit 20000 \
           -r "${START}-${END}" -o "$PART.tmp" "$URL"
      GOT=$([ -f "$PART.tmp" ] && stat -c %s "$PART.tmp" || echo 0)
      if [ "$GOT" -eq "$EXPECT" ]; then
        mv "$PART.tmp" "$PART"
        exit 0
      fi
      echo "[$(date +%H:%M:%S)] part $i 第 $attempt 次只拿到 $GOT/$EXPECT" >> "$LOG"
      # 若服务器支持断点，继续补；否则重下
      [ "$GOT" -gt 0 ] && { mv "$PART.tmp" "$PART"; START2=$(( START + GOT )); \
        curl -4 -sL --retry 5 --retry-delay 3 --retry-all-errors -r "${START2}-${END}" -o "$PART.tmp2" "$URL"; \
        cat "$PART" "$PART.tmp2" > "$PART.new" 2>/dev/null; mv "$PART.new" "$PART"; rm -f "$PART.tmp2"; }
      sleep 3
    done
    exit 1
  ) &
  pids+=($!)
done

log "已启动 ${#pids[@]} 个分片下载线程"
FAIL=0
for p in "${pids[@]}"; do wait "$p" || FAIL=1; done

if [ "$FAIL" -ne 0 ]; then
  log "!! 有分片失败，重跑本脚本可续传"
  exit 1
fi

# --- 3. 合并 ---
log "合并分片 -> $OUT"
: > "$OUT.tmp"
for ((i=0; i<PARTS; i++)); do
  PART="$PARTS_DIR/${BASE}.part$(printf '%03d' "$i")"
  [ -f "$PART" ] || break
  cat "$PART" >> "$OUT.tmp"
done
SIZE=$(stat -c %s "$OUT.tmp")
if [ "$SIZE" -eq "$TOTAL" ]; then
  mv "$OUT.tmp" "$OUT"
  log "完成：$OUT ($((SIZE/1048576)) MiB)"
  rm -rf "$PARTS_DIR"
  exit 0
else
  log "!! 合并后大小 $SIZE != $TOTAL，保留分片待续传"
  exit 1
fi

