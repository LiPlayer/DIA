colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  --base-paths src/ego-planner-swarm
