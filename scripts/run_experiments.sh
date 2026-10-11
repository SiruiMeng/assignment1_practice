#!/usr/bin/env bash
# Run each experiment: start the simulation and estimator, drive the square route,
# then shut everything down. One CSV per experiment is written to OUT_DIR.
# Usage: bash run_experiments.sh [OUT_DIR]
# Make sure no other Gazebo simulation is running first.
OUT_DIR=$(realpath -m "${1:-experiments}")
mkdir -p "$OUT_DIR"

# name | launch arguments
EXPERIMENTS=(
  "baseline|noise_k:=0.0 ticks_per_rev:=0"
  "noise_0005|noise_k:=0.005"
  "noise_001|noise_k:=0.01"
  "noise_002|noise_k:=0.02"
  "noise_005|noise_k:=0.05"
  "ticks_4096|ticks_per_rev:=4096"
  "ticks_360|ticks_per_rev:=360"
  "ticks_60|ticks_per_rev:=60"
  "radius_plus2pct|wheel_radius:=0.03366"
  "separation_plus5pct|wheel_separation:=0.168"
)

for exp in "${EXPERIMENTS[@]}"; do
  name=${exp%%|*}
  args=${exp#*|}
  # 已经有数据的实验跳过（删掉对应 CSV 就会重跑）
  if [ -f "$OUT_DIR/$name.csv" ] && [ "$(wc -l < "$OUT_DIR/$name.csv")" -gt 100 ]; then
    echo "=== $name: already done, skipping"
    continue
  fi
  echo "=== $name ($args)"
  ros2 launch odom_localization localization.launch.py $args \
    output_file:="$OUT_DIR/$name.csv" > "$OUT_DIR/$name.log" 2>&1 &
  launch_pid=$!
  sleep 15   # 等仿真启动、真值脚本运行（launch 里延迟 10 秒）
  # 超时保护：机器人没生成等异常情况下不会一直卡住
  timeout 300 ros2 run odom_localization drive_route_node --ros-args -p use_sim_time:=true
  sleep 3    # 让 logger 把车停下后的几行也记下来
  # 脚本里的后台进程会忽略 SIGINT，所以用 SIGTERM；15 秒内没退出就强制结束
  kill -TERM "$launch_pid" 2>/dev/null
  for _ in $(seq 15); do kill -0 "$launch_pid" 2>/dev/null || break; sleep 1; done
  kill -KILL "$launch_pid" 2>/dev/null
  # 真值 bridge 和 Gazebo 有时不会跟着 launch 一起退出，单独清理
  pkill -f "[p]arameter_bridge /ground_truth" 2>/dev/null
  pkill -9 -f "[g]z sim" 2>/dev/null   # Gazebo 会忽略普通的结束信号，必须 -9
  pkill -f "[p]arameter_bridge --ros-args -p config_file" 2>/dev/null
  pkill -f "[e]ncoder_sim_node|[o]dometry_node|[l]ogger_node|[r]obot_state_publisher" 2>/dev/null
  sleep 3
  echo "    rows: $(($(wc -l < "$OUT_DIR/$name.csv") - 1))"
done
