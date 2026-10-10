#!/usr/bin/env bash
# Attach Gazebo's OdometryPublisher to the robot (ground truth, evaluation only)
# and bridge it to ROS as /ground_truth/odom. Run after the simulation is up.
MODEL=${1:-burger}
ID=$(gz topic -e -t /world/default/pose/info -n 1 | grep -A1 "name: \"$MODEL\"" | awk '/id:/ {print $2}' | head -n1)
if [ -z "$ID" ]; then echo "Model $MODEL not found"; exit 1; fi
gz service -s /world/default/entity/system/add --reqtype gz.msgs.EntityPlugin_V \
  --reptype gz.msgs.Boolean --timeout 3000 \
  --req "entity: {id: $ID}, plugins: {name: \"gz::sim::systems::OdometryPublisher\", filename: \"gz-sim-odometry-publisher-system\", innerxml: \"<odom_topic>/ground_truth/odom</odom_topic><dimensions>2</dimensions><odom_publish_frequency>30</odom_publish_frequency>\"}"
exec ros2 run ros_gz_bridge parameter_bridge '/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'
