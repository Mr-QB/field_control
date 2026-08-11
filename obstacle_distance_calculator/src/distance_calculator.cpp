#include "obstacle_distance_calculator/distance_calculator.hpp"

namespace obstacle_distance_calculator
{

DistanceSummary DistanceCalculator::compute(
  const planning_scene::PlanningSceneConstPtr & scene,
  const moveit::core::RobotState & robot_state,
  const CalculatorOptions & options)
{
  DistanceSummary summary;
  if (!scene) {
    return summary;
  }

  collision_detection::DistanceRequest req;
  req.enable_nearest_points = true;
  req.enable_signed_distance = true;
  req.compute_gradient = true;
  req.type = collision_detection::DistanceRequestTypes::SINGLE;
  req.distance_threshold = options.distance_threshold;
  req.acm = &scene->getAllowedCollisionMatrix();

  if (!options.group_name.empty()) {
    req.group_name = options.group_name;
    req.enableGroup(scene->getRobotModel());
  }

  collision_detection::DistanceResult res;
  const auto & env = options.use_unpadded_env ?
    scene->getCollisionEnvUnpadded() :
    scene->getCollisionEnv();

  if (!env) {
    return summary;
  }

  env->distanceRobot(req, res, robot_state);

  const auto & robot_model = scene->getRobotModel();

  for (const auto & entry : res.distances) {
    for (const auto & dist_data : entry.second) {
      if (dist_data.distance > options.distance_threshold) {
        continue;
      }

      std::string link_name;
      std::string obstacle_id;
      Eigen::Vector3d pt_link;
      Eigen::Vector3d pt_obs;
      Eigen::Vector3d normal_vec;

      bool is_0_link = robot_model && robot_model->hasLinkModel(dist_data.link_names[0]);
      bool is_1_link = robot_model && robot_model->hasLinkModel(dist_data.link_names[1]);

      if (is_0_link && !is_1_link) {
        link_name = dist_data.link_names[0];
        obstacle_id = dist_data.link_names[1];
        pt_link = dist_data.nearest_points[0];
        pt_obs = dist_data.nearest_points[1];
        normal_vec = dist_data.normal;
      } else if (is_1_link && !is_0_link) {
        link_name = dist_data.link_names[1];
        obstacle_id = dist_data.link_names[0];
        pt_link = dist_data.nearest_points[1];
        pt_obs = dist_data.nearest_points[0];
        normal_vec = -dist_data.normal;  // Flip vector to point link -> obstacle
      } else if (
        dist_data.body_types[0] == collision_detection::BodyTypes::ROBOT_LINK ||
        dist_data.body_types[0] == collision_detection::BodyTypes::ROBOT_ATTACHED)
      {
        link_name = dist_data.link_names[0];
        obstacle_id = dist_data.link_names[1];
        pt_link = dist_data.nearest_points[0];
        pt_obs = dist_data.nearest_points[1];
        normal_vec = dist_data.normal;
      } else if (
        dist_data.body_types[1] == collision_detection::BodyTypes::ROBOT_LINK ||
        dist_data.body_types[1] == collision_detection::BodyTypes::ROBOT_ATTACHED)
      {
        link_name = dist_data.link_names[1];
        obstacle_id = dist_data.link_names[0];
        pt_link = dist_data.nearest_points[1];
        pt_obs = dist_data.nearest_points[0];
        normal_vec = -dist_data.normal;
      } else {
        continue;
      }

      // Ensure normal vector is normalized and valid
      if (normal_vec.norm() < 1e-6) {
        Eigen::Vector3d diff = pt_obs - pt_link;
        if (diff.norm() > 1e-6) {
          normal_vec = diff.normalized();
        } else {
          normal_vec.setZero();
        }
      } else {
        normal_vec.normalize();
      }

      DistanceItem item;
      item.link_name = link_name;
      item.obstacle_id = obstacle_id;
      item.distance = dist_data.distance;
      item.point_on_link = pt_link;
      item.point_on_obstacle = pt_obs;
      item.normal = normal_vec;

      summary.items.push_back(item);

      if (dist_data.distance < summary.overall_min_distance) {
        summary.overall_min_distance = dist_data.distance;
        summary.overall_closest_link = link_name;
        summary.overall_closest_obstacle = obstacle_id;
      }
    }
  }

  return summary;
}

}  // namespace obstacle_distance_calculator
