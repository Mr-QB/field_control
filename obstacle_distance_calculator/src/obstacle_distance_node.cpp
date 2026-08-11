#include "obstacle_distance_calculator/obstacle_distance_node.hpp"

namespace obstacle_distance_calculator
{

ObstacleDistanceNode::ObstacleDistanceNode(const rclcpp::NodeOptions & options)
: Node("obstacle_distance_node", options)
{
  robot_description_ = declare_parameter<std::string>("robot_description", "robot_description");
  group_name_ = declare_parameter<std::string>("group_name", "");
  output_topic_ = declare_parameter<std::string>("output_topic", "obstacle_distances");
  publish_frequency_ = declare_parameter<double>("publish_frequency", 50.0);
  distance_threshold_ = declare_parameter<double>("distance_threshold", 0.3);
  use_unpadded_env_ = declare_parameter<bool>("use_unpadded_env", true);

  distance_pub_ = create_publisher<obstacle_distance_msgs::msg::LinkObstacleDistanceArray>(
    output_topic_, 10);
}

void ObstacleDistanceNode::init()
{
  psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
    shared_from_this(), robot_description_, "planning_scene_monitor");

  if (psm_->getPlanningScene()) {
    psm_->startStateMonitor();
    psm_->startWorldGeometryMonitor();
    psm_->startSceneMonitor();
    RCLCPP_INFO(get_logger(), "PlanningSceneMonitor initialized successfully.");
  } else {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to initialize PlanningSceneMonitor. Check parameter '%s'.",
      robot_description_.c_str());
  }

  auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_frequency_));
  timer_ = create_wall_timer(period, std::bind(&ObstacleDistanceNode::timer_callback, this));
}

void ObstacleDistanceNode::timer_callback()
{
  if (!psm_) {
    return;
  }

  planning_scene_monitor::LockedPlanningSceneRO locked_scene(psm_);
  if (!locked_scene) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "PlanningScene is locked or unavailable.");
    return;
  }

  CalculatorOptions options;
  options.distance_threshold = distance_threshold_;
  options.group_name = group_name_;
  options.use_unpadded_env = use_unpadded_env_;

  const auto & robot_state = locked_scene->getCurrentState();
  DistanceSummary summary = DistanceCalculator::compute(locked_scene, robot_state, options);

  auto array_msg = obstacle_distance_msgs::msg::LinkObstacleDistanceArray();
  array_msg.header.stamp = now();
  array_msg.header.frame_id = locked_scene->getPlanningFrame();

  for (const auto & item : summary.items) {
    obstacle_distance_msgs::msg::LinkObstacleDistance link_dist_msg;
    link_dist_msg.link_name = item.link_name;
    link_dist_msg.obstacle_id = item.obstacle_id;
    link_dist_msg.distance = item.distance;

    link_dist_msg.point_on_link.x = item.point_on_link.x();
    link_dist_msg.point_on_link.y = item.point_on_link.y();
    link_dist_msg.point_on_link.z = item.point_on_link.z();

    link_dist_msg.point_on_obstacle.x = item.point_on_obstacle.x();
    link_dist_msg.point_on_obstacle.y = item.point_on_obstacle.y();
    link_dist_msg.point_on_obstacle.z = item.point_on_obstacle.z();

    link_dist_msg.normal.x = item.normal.x();
    link_dist_msg.normal.y = item.normal.y();
    link_dist_msg.normal.z = item.normal.z();

    array_msg.distances.push_back(link_dist_msg);
  }

  array_msg.overall_min_distance = summary.overall_min_distance;
  array_msg.overall_closest_link = summary.overall_closest_link;
  array_msg.overall_closest_obstacle = summary.overall_closest_obstacle;

  distance_pub_->publish(array_msg);
}

}  // namespace obstacle_distance_calculator
