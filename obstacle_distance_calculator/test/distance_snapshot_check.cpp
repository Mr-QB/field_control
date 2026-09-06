// Read-only live check: fetch one scene and calculate locally; publish nothing.
#include "obstacle_distance_calculator/distance_calculator.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("distance_snapshot_check",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  robot_model_loader::RobotModelLoader loader(node, "robot_description", false);
  if (!loader.getModel()) {return 1;}
  auto client = node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  if (!client->wait_for_service(std::chrono::seconds(5))) {return 2;}
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components = 1023;
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(5)) !=
      rclcpp::FutureReturnCode::SUCCESS) {return 3;}
  auto scene = std::make_shared<planning_scene::PlanningScene>(loader.getModel());
  if (!scene->setPlanningSceneMsg(future.get()->scene)) {return 4;}
  collision_detection::CollisionRequest collision_request;
  collision_request.contacts = true;
  collision_request.max_contacts = 100;
  collision_detection::CollisionResult collision_result;
  scene->getCollisionEnvUnpadded()->checkRobotCollision(collision_request, collision_result,
    scene->getCurrentState(), scene->getAllowedCollisionMatrix());
  obstacle_distance_calculator::CalculatorOptions options;
  options.distance_threshold = 1.0;
  const auto start = std::chrono::steady_clock::now();
  const auto result = obstacle_distance_calculator::DistanceCalculator::compute(scene, scene->getCurrentState(), options);
  const double milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  std::cout << std::setprecision(10) << "frame=" << scene->getPlanningFrame()
    << " collision=" << collision_result.collision << " minimum=" << result.overall_min_distance
    << " pair=" << result.overall_closest_link << "/" << result.overall_closest_obstacle
    << " compute_ms=" << milliseconds << '\n';
  double max_witness_error = 0.0;
  for (const auto & item : result.items) {
    const double error = ((item.point_on_obstacle - item.point_on_link) - item.distance * item.normal).norm();
    max_witness_error = std::max(max_witness_error, error);
    std::cout << item.link_name << '/' << item.obstacle_id << " d=" << item.distance
      << " link_point=" << item.point_on_link.transpose()
      << " obstacle_point=" << item.point_on_obstacle.transpose() << '\n';
  }
  std::cout << "maximum_witness_error=" << max_witness_error << '\n';
  // Sweep a sphere across the measured surface in this LOCAL scene only.
  // Neither ApplyPlanningScene nor any publisher is used by this diagnostic.
  const auto closest = std::find_if(result.items.begin(), result.items.end(), [&](const auto & item) {
    return item.link_name == result.overall_closest_link && item.obstacle_id == result.overall_closest_obstacle;
  });
  if (closest != result.items.end()) {
    const auto object = scene->getWorld()->getObject(closest->obstacle_id);
    if (object && object->shapes_.size() == 1 && object->shapes_[0]->type == shapes::SPHERE) {
      const double radius = static_cast<const shapes::Sphere &>(*object->shapes_[0]).radius;
      scene->getWorldNonConst()->clearObjects();
      for (const auto * link : scene->getRobotModel()->getLinkModels()) {
        if (link->getName() != closest->link_name) {
          scene->getAllowedCollisionMatrixNonConst().setEntry(link->getName(), "local_probe", true);
        }
      }
      for (double gap : {0.02, 0.001, 0.0, -0.001, -0.02}) {
        scene->getWorldNonConst()->removeObject("local_probe");
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = closest->point_on_link + closest->normal * (radius + gap);
        scene->getWorldNonConst()->addToObject("local_probe", std::make_shared<shapes::Sphere>(radius), pose);
        collision_result.clear();
        scene->getCollisionEnvUnpadded()->checkRobotCollision(collision_request, collision_result,
          scene->getCurrentState(), scene->getAllowedCollisionMatrix());
        const auto swept = obstacle_distance_calculator::DistanceCalculator::compute(scene, scene->getCurrentState(), options);
        std::cout << "local_sweep expected=" << gap << " measured=" << swept.overall_min_distance
          << " collision=" << collision_result.collision << '\n';
        if (std::abs(swept.overall_min_distance - gap) > 1e-5) {return 6;}
      }
    }
  }
  rclcpp::shutdown();
  return max_witness_error < 1e-6 ? 0 : 5;
}
