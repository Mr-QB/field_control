#ifndef OBSTACLE_DISTANCE_CALCULATOR__OBSTACLE_DISTANCE_NODE_HPP_
#define OBSTACLE_DISTANCE_CALCULATOR__OBSTACLE_DISTANCE_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <obstacle_distance_msgs/msg/link_obstacle_distance.hpp>
#include <obstacle_distance_msgs/msg/link_obstacle_distance_array.hpp>
#include "obstacle_distance_calculator/distance_calculator.hpp"

namespace obstacle_distance_calculator
{

class ObstacleDistanceNode : public rclcpp::Node
{
public:
  explicit ObstacleDistanceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~ObstacleDistanceNode() = default;

  /// Initialize PlanningSceneMonitor and start monitoring
  void init();

private:
  /// Timer callback to execute calculation and publish ROS 2 message
  void timer_callback();

  // Parameters
  std::string robot_description_;
  std::string group_name_;
  std::string output_topic_;
  double publish_frequency_;
  double distance_threshold_;
  bool use_unpadded_env_;

  // MoveIt PlanningSceneMonitor
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;

  // ROS 2 Publisher & Timer
  rclcpp::Publisher<obstacle_distance_msgs::msg::LinkObstacleDistanceArray>::SharedPtr distance_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace obstacle_distance_calculator

#endif  // OBSTACLE_DISTANCE_CALCULATOR__OBSTACLE_DISTANCE_NODE_HPP_
