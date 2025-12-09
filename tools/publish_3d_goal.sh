#!/usr/bin/env bash
# Tiny helper to publish a 3D goal pose (map frame) to /ego_planner/single_target

set -euo pipefail

if [[ $# -lt 4 ]]; then
echo "Usage: $0 <x> <y> <z> <yaw_deg>"
echo "Example: $0 1.0 2.0 0.8 90"
  exit 1
fi

X=$1
Y=$2
Z=$3
YAW_DEG=$4
read QX QY QZ QW <<<"$(python3 - "$YAW_DEG" <<'PY'
import sys, math
yaw_deg = float(sys.argv[1])
yaw = math.radians(yaw_deg)
half = yaw * 0.5
qx = 0.0
qy = 0.0
qz = math.sin(half)
qw = math.cos(half)
print(f"{qx} {qy} {qz} {qw}")
PY
)"

ros2 topic pub --once /ego_planner/single_target geometry_msgs/msg/PoseStamped "
header:
  frame_id: map
pose:
  position: {x: ${X}, y: ${Y}, z: ${Z}}
  orientation: {x: ${QX}, y: ${QY}, z: ${QZ}, w: ${QW}}
"
