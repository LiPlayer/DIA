#include <rclcpp/rclcpp.hpp>

#include "trajectory_control/controller.hpp"
#include "trajectory_control/vio_bridge.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto controller = std::make_shared<trajectory_control::TrajController>();
  auto vio_bridge = std::make_shared<trajectory_control::VioBridge>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller);
  executor.add_node(vio_bridge);

  RCLCPP_WARN(controller->get_logger(),
              "[Traj Server PX4]: controller + VIO bridge ready.");

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
