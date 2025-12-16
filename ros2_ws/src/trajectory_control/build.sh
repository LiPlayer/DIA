#!/bin/bash
cd "$(dirname "$0")/../.." || exit 1

colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  --packages-select trajectory_control
