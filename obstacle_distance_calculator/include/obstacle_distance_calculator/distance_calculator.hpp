#ifndef OBSTACLE_DISTANCE_CALCULATOR__DISTANCE_CALCULATOR_HPP_
#define OBSTACLE_DISTANCE_CALCULATOR__DISTANCE_CALCULATOR_HPP_

#include <string>
#include <vector>
#include <limits>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_env.h>
#include <moveit/collision_detection/collision_common.h>

namespace obstacle_distance_calculator
{

struct DistanceItem
{
  std::string link_name;
  std::string obstacle_id;
  double distance;
  Eigen::Vector3d point_on_link;
  Eigen::Vector3d point_on_obstacle;
  Eigen::Vector3d normal;
};

struct DistanceSummary
{
  std::vector<DistanceItem> items;
  double overall_min_distance{std::numeric_limits<double>::infinity()};
  std::string overall_closest_link{""};
  std::string overall_closest_obstacle{""};
};

struct CalculatorOptions
{
  double distance_threshold{0.5};
  std::string group_name{""};
  bool use_unpadded_env{true};
};

class DistanceCalculator
{
public:
  DistanceCalculator() = default;
  ~DistanceCalculator() = default;

  /// Calculate robot-to-world clearance from PlanningScene and RobotState.
  ///
  /// Positive = separated, zero = touching, negative = overlap (meters).
  /// Mesh/sphere pairs use signed centre-to-mesh distance minus sphere radius;
  /// other shapes use MoveIt's signed distance backend. Containment assumes
  /// closed meshes. Points and normals are expressed in the planning frame.
  /// If no pair is within distance_threshold, minimum is +infinity and names
  /// are empty. Self-collision is outside this calculator's scope.
  static DistanceSummary compute(
    const planning_scene::PlanningSceneConstPtr & scene,
    const moveit::core::RobotState & robot_state,
    const CalculatorOptions & options);
};

}  // namespace obstacle_distance_calculator

#endif  // OBSTACLE_DISTANCE_CALCULATOR__DISTANCE_CALCULATOR_HPP_
