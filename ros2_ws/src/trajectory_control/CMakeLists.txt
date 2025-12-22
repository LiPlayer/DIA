cmake_minimum_required(VERSION 3.8)
project(trajectory_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(traj_utils REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(trajectory_controller
  src/main.cpp
  src/controller.cpp
  src/vio_bridge.cpp
)
ament_target_dependencies(trajectory_controller
  rclcpp
  px4_msgs
  traj_utils
  bspline_opt
  nav_msgs
)
target_include_directories(trajectory_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

install(TARGETS trajectory_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
