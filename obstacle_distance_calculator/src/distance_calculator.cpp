#include "obstacle_distance_calculator/distance_calculator.hpp"

namespace obstacle_distance_calculator
{

DistanceSummary DistanceCalculator::compute(
  const planning_scene::PlanningSceneConstPtr & scene,
  const moveit::core::RobotState & robot_state,
  const CalculatorOptions & options)
{
  DistanceSummary summary;

  // Yêu cầu MoveIt trả về khoảng cách, hai điểm gần nhất và vector pháp tuyến.
  collision_detection::DistanceRequest req;
  req.enable_nearest_points = true;
  req.enable_signed_distance = true;
  req.compute_gradient = true;
  req.type = collision_detection::DistanceRequestTypes::SINGLE;
  req.distance_threshold = options.distance_threshold;
  req.acm = &scene->getAllowedCollisionMatrix();

  if (!options.group_name.empty()) {
    // Nếu có group_name, chỉ xét các link thuộc nhóm đó.
    req.group_name = options.group_name;
    req.enableGroup(scene->getRobotModel());
  }

  // Chọn collision environment có hoặc không có padding tùy cấu hình.
  const auto & env = options.use_unpadded_env ?
    scene->getCollisionEnvUnpadded() :
    scene->getCollisionEnv();
  collision_detection::DistanceResult res;
  env->distanceRobot(req, res, robot_state);

  // Mỗi entry là một cặp body nằm trong khoảng distance_threshold.
  for (const auto & entry : res.distances) {
    for (const auto & dist_data : entry.second) {
      // Xác định body nào trong cặp là robot. Body còn lại là obstacle.
      const bool robot_is_first =
        dist_data.body_types[0] == collision_detection::BodyTypes::ROBOT_LINK ||
        dist_data.body_types[0] == collision_detection::BodyTypes::ROBOT_ATTACHED;
      const bool robot_is_second =
        dist_data.body_types[1] == collision_detection::BodyTypes::ROBOT_LINK ||
        dist_data.body_types[1] == collision_detection::BodyTypes::ROBOT_ATTACHED;
      if (!robot_is_first && !robot_is_second) {
        continue;
      }

      DistanceItem item;
      // Đổi chỉ số để dữ liệu đầu ra luôn theo thứ tự: robot -> obstacle.
      const auto link_index = robot_is_first ? 0 : 1;
      const auto obstacle_index = robot_is_first ? 1 : 0;
      item.link_name = dist_data.link_names[link_index];
      item.obstacle_id = dist_data.link_names[obstacle_index];
      item.distance = dist_data.distance;
      item.point_on_link = dist_data.nearest_points[link_index];
      item.point_on_obstacle = dist_data.nearest_points[obstacle_index];
      // Đảo chiều normal khi robot nằm ở body thứ hai để nó luôn chỉ từ link sang obstacle.
      item.normal = robot_is_first ? dist_data.normal : -dist_data.normal;

      summary.items.push_back(item);

      if (dist_data.distance < summary.overall_min_distance) {
        // Lưu lại cặp gần nhất trong tất cả các cặp đã tính.
        summary.overall_min_distance = dist_data.distance;
        summary.overall_closest_link = item.link_name;
        summary.overall_closest_obstacle = item.obstacle_id;
      }
    }
  }

  return summary;
}

}  // namespace obstacle_distance_calculator
