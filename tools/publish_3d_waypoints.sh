#!/usr/bin/env bash
set -e

# Publish a PoseArray of waypoints (previous defaults from advanced_param.launch.py)
# to /ego_planner/waypoints_target.

WS_ROOT="$(dirname "$(dirname "$(readlink -f "$0")")")"

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  source "$WS_ROOT/install/setup.bash"
fi

ros2 topic pub --once /ego_planner/waypoints_target geometry_msgs/msg/PoseArray "{
  header: {frame_id: 'world'},
  poses: [
    {position: {x: 20.0, y: 10.0, z: 0.0}, orientation: {w: 1.0}},
    {position: {x: -20.0, y: 10.0, z: 2.0}, orientation: {w: 1.0}},
    {position: {x: 20.0, y: -10.0, z: 0.0}, orientation: {w: 1.0}},
    {position: {x: -20.0, y: -10.0, z: 2.0}, orientation: {w: 1.0}},
    {position: {x: 0.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}
  ]
}"
