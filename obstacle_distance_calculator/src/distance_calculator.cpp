#include "obstacle_distance_calculator/distance_calculator.hpp"
#include "mesh_sphere_distance.hpp"

#include <cmath>
#include <map>
#include <stdexcept>

namespace obstacle_distance_calculator
{
namespace
{
struct Body
{
  // Keep MoveIt's geometry metadata alive throughout the query.
  collision_detection::FCLGeometryConstPtr geometry;
  std::shared_ptr<fcl::CollisionObjectd> object;
};

void appendBody(std::vector<Body> & bodies,
  const collision_detection::FCLGeometryConstPtr & geometry, const Eigen::Isometry3d & pose)
{
  if (!geometry) {
    throw std::runtime_error("Cannot construct collision geometry for distance query");
  }
  bodies.push_back({geometry, std::make_shared<fcl::CollisionObjectd>(geometry->collision_geometry_, pose)});
}
}  // namespace

DistanceSummary DistanceCalculator::compute(
  const planning_scene::PlanningSceneConstPtr & scene,
  const moveit::core::RobotState & robot_state,
  const CalculatorOptions & options)
{
  if (!scene || !std::isfinite(options.distance_threshold) || options.distance_threshold < 0.0) {
    throw std::invalid_argument("A scene and finite, nonnegative distance_threshold are required");
  }
  const auto & model = scene->getRobotModel();
  const auto * group = options.group_name.empty() ? nullptr : model->getJointModelGroup(options.group_name);
  if (!options.group_name.empty() && !group) {
    throw std::invalid_argument("Unknown joint group: " + options.group_name);
  }
  // Refresh collision transforms even if the caller only set joint positions.
  moveit::core::RobotState state(robot_state);
  state.update();
  const auto & env = options.use_unpadded_env ? scene->getCollisionEnvUnpadded() : scene->getCollisionEnv();
  const auto active = [group](const moveit::core::LinkModel * link) {
    return !group || group->getUpdatedLinkModelsSet().count(link) != 0;
  };

  std::vector<Body> robot_bodies;
  for (const auto * link : model->getLinkModelsWithCollisionGeometry()) {
    if (!active(link)) {
      continue;
    }
    for (std::size_t i = 0; i < link->getShapes().size(); ++i) {
      appendBody(robot_bodies, collision_detection::createCollisionGeometry(
        link->getShapes()[i], env->getLinkScale(link->getName()),
        env->getLinkPadding(link->getName()), link, i), state.getCollisionBodyTransform(link, i));
    }
  }
  std::vector<const moveit::core::AttachedBody *> attached_bodies;
  state.getAttachedBodies(attached_bodies);
  for (const auto * attached : attached_bodies) {
    if (!active(attached->getAttachedLink())) {
      continue;
    }
    for (std::size_t i = 0; i < attached->getShapes().size(); ++i) {
      appendBody(robot_bodies, collision_detection::createCollisionGeometry(
        attached->getShapes()[i], env->getLinkScale(attached->getAttachedLinkName()),
        env->getLinkPadding(attached->getAttachedLinkName()), attached, i),
        attached->getGlobalCollisionBodyTransforms()[i]);
    }
  }
  std::vector<Body> world_bodies;
  for (const auto & entry : *scene->getWorld()) {
    const auto & object = entry.second;
    for (std::size_t i = 0; i < object->shapes_.size(); ++i) {
      appendBody(world_bodies, collision_detection::createCollisionGeometry(object->shapes_[i], object.get()),
        object->global_shape_poses_[i]);
    }
  }

  collision_detection::DistanceRequest request;
  request.type = collision_detection::DistanceRequestTypes::SINGLE;
  request.enable_nearest_points = true;
  request.enable_signed_distance = true;
  request.compute_gradient = true;
  request.distance_threshold = options.distance_threshold;
  request.acm = &scene->getAllowedCollisionMatrix();

  // Reduce shape pairs to one minimum per robot-body/world-object pair.
  std::map<std::pair<std::string, std::string>, DistanceItem> pair_minima;
  for (const auto & robot : robot_bodies) {
    for (const auto & world : world_bodies) {
      const auto & robot_id = robot.geometry->collision_geometry_data_->getID();
      const auto & obstacle_id = world.geometry->collision_geometry_data_->getID();
      collision_detection::AllowedCollision::Type allowed;
      if (request.acm->getAllowedCollision(robot_id, obstacle_id, allowed) &&
          allowed == collision_detection::AllowedCollision::ALWAYS) {
        continue;
      }
      if (robot.object->getAABB().distance(world.object->getAABB()) > options.distance_threshold) {
        continue;
      }

      collision_detection::DistanceResultsData distance;
      if (!detail::meshSphereDistance(*robot.object, *world.object, distance)) {
        // Other shapes use MoveIt's signed narrow-phase query.
        collision_detection::DistanceResult result;
        collision_detection::DistanceData data(&request, &result);
        double unused_threshold = options.distance_threshold;
        collision_detection::distanceCallback(robot.object.get(), world.object.get(), &data, unused_threshold);
        distance = result.minimum_distance;
      }
      if (distance.link_names[0].empty() || distance.link_names[1].empty()) {
        continue;
      }
      if (!std::isfinite(distance.distance) || !distance.nearest_points[0].allFinite() ||
          !distance.nearest_points[1].allFinite() || !distance.normal.allFinite()) {
        throw std::runtime_error("Invalid distance result for " + robot_id + " / " + obstacle_id);
      }
      if (distance.distance > options.distance_threshold) {
        continue;
      }
      const int robot_index = distance.body_types[0] == collision_detection::BodyTypes::WORLD_OBJECT ? 1 : 0;
      DistanceItem item;
      item.link_name = distance.link_names[robot_index];
      item.obstacle_id = distance.link_names[1 - robot_index];
      item.distance = distance.distance;
      item.point_on_link = distance.nearest_points[robot_index];
      item.point_on_obstacle = distance.nearest_points[1 - robot_index];
      item.normal = robot_index == 0 ? distance.normal : -distance.normal;
      const auto key = std::make_pair(item.link_name, item.obstacle_id);
      const auto existing = pair_minima.find(key);
      if (existing == pair_minima.end() || item.distance < existing->second.distance) {
        pair_minima[key] = item;
      }
    }
  }
  DistanceSummary summary;
  for (const auto & pair : pair_minima) {
    const auto & item = pair.second;
    summary.items.push_back(item);
    if (item.distance < summary.overall_min_distance) {
      summary.overall_min_distance = item.distance;
      summary.overall_closest_link = item.link_name;
      summary.overall_closest_obstacle = item.obstacle_id;
    }
  }
  return summary;
}
}  // namespace obstacle_distance_calculator
