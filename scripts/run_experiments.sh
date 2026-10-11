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
  # Skip experiments that already have data (delete the CSV to rerun)
  if [ -f "$OUT_DIR/$name.csv" ] && [ "$(wc -l < "$OUT_DIR/$name.csv")" -gt 100 ]; then
    echo "=== $name: already done, skipping"
    continue
  fi
  echo "=== $name ($args)"
  ros2 launch odom_localization localization.launch.py $args \
    output_file:="$OUT_DIR/$name.csv" > "$OUT_DIR/$name.log" 2>&1 &
  launch_pid=$!
  sleep 15   # wait for Gazebo and the ground-truth script (started after 10 s)
  # Timeout so a failed spawn cannot hang the whole batch
  timeout 300 ros2 run odom_localization drive_route_node --ros-args -p use_sim_time:=true
  sleep 3    # log a few rows after the robot has stopped
  # Background jobs in scripts ignore SIGINT, so use SIGTERM; force-kill after 15 s
  kill -TERM "$launch_pid" 2>/dev/null
  for _ in $(seq 15); do kill -0 "$launch_pid" 2>/dev/null || break; sleep 1; done
  kill -KILL "$launch_pid" 2>/dev/null
  # The ground-truth bridge and Gazebo do not always exit with the launch
  pkill -f "[p]arameter_bridge /ground_truth" 2>/dev/null
  pkill -9 -f "[g]z sim" 2>/dev/null   # Gazebo ignores SIGTERM
  pkill -f "[p]arameter_bridge --ros-args -p config_file" 2>/dev/null
  pkill -f "[e]ncoder_sim_node|[o]dometry_node|[l]ogger_node|[r]obot_state_publisher" 2>/dev/null
  sleep 3
  echo "    rows: $(($(wc -l < "$OUT_DIR/$name.csv") - 1))"
done
