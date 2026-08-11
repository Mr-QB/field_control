#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "obstacle_distance_calculator/obstacle_distance_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<obstacle_distance_calculator::ObstacleDistanceNode>(options);
  node->init();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
