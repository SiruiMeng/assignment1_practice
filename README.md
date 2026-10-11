# odom_localization

Wheel-odometry (dead-reckoning) localization of a TurtleBot3 Burger in Gazebo, with
simulated encoder uncertainty, evaluated against the simulator's ground truth.
Intelligent Robotics, Assignment 1.

## Data flow

```
Gazebo /joint_states -> encoder_sim_node -> /encoder/joint_states -> odometry_node
       -> /odometry/estimate -> logger_node (+ /ground_truth/odom) -> CSV
```

The estimator reads the wheel encoders only. Ground truth is used for evaluation only.

## Requirements

- ROS 2 Jazzy with Gazebo (`gz sim`) and `ros_gz_bridge`
- `turtlebot3_simulations` (jazzy branch) in the same workspace
- Python packages `numpy` and `matplotlib` for the analysis scripts

## Build

```bash
cd <workspace>            # contains src/odom_localization and src/turtlebot3_simulations
colcon build
source install/setup.bash
```

## Run one experiment

```bash
# terminal 1: simulation, ground truth, encoder simulation, estimator and logger
ros2 launch odom_localization localization.launch.py noise_k:=0.05 output_file:=run.csv

# terminal 2: drive the 1 m square, then stop terminal 1 with Ctrl+C
ros2 run odom_localization drive_route_node --ros-args -p use_sim_time:=true
```

Launch arguments:

| Argument | Default | Meaning |
|---|---|---|
| `noise_k` | `0.0` | Random encoder noise, sigma = noise_k * sqrt(\|dphi\|) |
| `ticks_per_rev` | `0` | Encoder resolution in ticks per revolution (0 = no quantization) |
| `wheel_radius` | `0.033` | Wheel radius assumed by the estimator [m] |
| `wheel_separation` | `0.160` | Wheel separation assumed by the estimator [m] |
| `output_file` | `data.csv` | CSV written by the logger |

## Run all experiments and plot

```bash
bash src/odom_localization/scripts/run_experiments.sh experiments
python3 src/odom_localization/scripts/analyze.py experiments/*.csv --out results
python3 src/odom_localization/scripts/submission_plot.py experiments trajectory_vs_ground_truth
```

## Files

| File | Purpose |
|---|---|
| `odom_localization/odometry_math.py` | Differential-drive odometry equations (Lecture 2) |
| `odom_localization/odometry_node.py` | Pose estimator |
| `odom_localization/encoder_sim_node.py` | Encoder noise and quantization model |
| `odom_localization/logger_node.py` | Writes estimate and ground truth to CSV |
| `odom_localization/drive_route_node.py` | Drives a repeatable square route |
| `scripts/ground_truth.sh` | Attaches Gazebo's OdometryPublisher and bridges it to ROS |
| `scripts/run_experiments.sh` | Runs every experiment in turn |
| `scripts/analyze.py`, `scripts/submission_plot.py` | Error metrics and figures |
| `launch/localization.launch.py` | Starts everything for one run |
