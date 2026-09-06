#include "obstacle_distance_calculator/distance_calculator.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <geometric_shapes/shapes.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

namespace obstacle_distance_calculator
{
namespace
{

// Analytic geometry makes these expectations independent of the distance
// backend: two spheres of radius 0.1 m have distance |c1 - c2| - 0.2 m.
class DistanceCalculatorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const std::string urdf = R"(
      <robot name="distance_test_robot">
        <link name="world"/>
        <link name="base_link">
          <collision><geometry><sphere radius="0.1"/></geometry></collision>
        </link>
        <joint name="robot_pose" type="floating">
          <parent link="world"/><child link="base_link"/>
        </joint>
        <link name="tip_link">
          <collision><geometry><sphere radius="0.1"/></geometry></collision>
        </link>
        <joint name="tip_slide" type="prismatic">
          <parent link="base_link"/><child link="tip_link"/>
          <origin xyz="0 2 0"/><axis xyz="1 0 0"/>
          <limit lower="-1" upper="1" effort="10" velocity="1"/>
        </joint>
      </robot>)";
    const std::string srdf = R"(
      <robot name="distance_test_robot">
        <group name="tip_only"><joint name="tip_slide"/></group>
      </robot>)";

    const auto urdf_model = urdf::parseURDF(urdf);
    ASSERT_NE(urdf_model, nullptr);
    auto srdf_model = std::make_shared<srdf::Model>();
    ASSERT_TRUE(srdf_model->initString(*urdf_model, srdf));
    scene_ = std::make_shared<planning_scene::PlanningScene>(urdf_model, srdf_model);
    state_ = std::make_unique<moveit::core::RobotState>(scene_->getRobotModel());
    state_->setToDefaultValues();
    state_->update();
  }

  void addSphere(const std::string & id, const Eigen::Vector3d & center, double radius = 0.1)
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = center;
    scene_->getWorldNonConst()->addToObject(
      id, std::make_shared<shapes::Sphere>(radius), pose);
  }

  void addBox(const std::string & id, const Eigen::Vector3d & center)
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = center;
    scene_->getWorldNonConst()->addToObject(
      id, std::make_shared<shapes::Box>(0.2, 0.2, 0.2), pose);
  }

  void addMeshBox(const std::string & id, const Eigen::Vector3d & center, double yaw = 0.0)
  {
    // The same 0.2 m cube, represented by triangles to exercise the
    // mesh/primitive FCL path used by UR robot collision geometry.
    const double vertices[] = {
      -0.1, -0.1, -0.1, 0.1, -0.1, -0.1, 0.1, 0.1, -0.1, -0.1, 0.1, -0.1,
      -0.1, -0.1, 0.1, 0.1, -0.1, 0.1, 0.1, 0.1, 0.1, -0.1, 0.1, 0.1};
    const unsigned int triangles[] = {
      0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7,
      0, 1, 5, 0, 5, 4, 3, 7, 6, 3, 6, 2,
      0, 4, 7, 0, 7, 3, 1, 2, 6, 1, 6, 5};
    auto mesh = std::make_shared<shapes::Mesh>(8, 12);
    std::copy(std::begin(vertices), std::end(vertices), mesh->vertices);
    std::copy(std::begin(triangles), std::end(triangles), mesh->triangles);
    mesh->computeTriangleNormals();
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = center;
    pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    scene_->getWorldNonConst()->addToObject(id, mesh, pose);
  }

  void moveRobot(const Eigen::Vector3d & translation, double yaw = 0.0)
  {
    const Eigen::Quaterniond rotation(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    const std::vector<double> positions{
      translation.x(), translation.y(), translation.z(),
      rotation.x(), rotation.y(), rotation.z(), rotation.w()};
    state_->setJointPositions("robot_pose", positions);
    state_->update();
  }

  DistanceSummary compute() const
  {
    return DistanceCalculator::compute(scene_, *state_, options_);
  }

  void expectMinimum(
    const DistanceSummary & summary, double expected,
    const std::string & link = "base_link", const std::string & obstacle = "obstacle") const
  {
    ASSERT_FALSE(summary.items.empty());
    EXPECT_NEAR(summary.overall_min_distance, expected, 1e-5);
    EXPECT_EQ(summary.overall_closest_link, link);
    EXPECT_EQ(summary.overall_closest_obstacle, obstacle);

    const auto minimum = std::min_element(
      summary.items.begin(), summary.items.end(),
      [](const DistanceItem & first, const DistanceItem & second) {
        return first.distance < second.distance;
      });
    EXPECT_NEAR(summary.overall_min_distance, minimum->distance, 1e-8);
    EXPECT_EQ(summary.overall_closest_link, minimum->link_name);
    EXPECT_EQ(summary.overall_closest_obstacle, minimum->obstacle_id);
  }

  void expectNoPairs(const DistanceSummary & summary) const
  {
    EXPECT_TRUE(summary.items.empty());
    EXPECT_TRUE(std::isinf(summary.overall_min_distance));
    EXPECT_GT(summary.overall_min_distance, 0.0);
    EXPECT_TRUE(summary.overall_closest_link.empty());
    EXPECT_TRUE(summary.overall_closest_obstacle.empty());
  }

  planning_scene::PlanningScenePtr scene_;
  std::unique_ptr<moveit::core::RobotState> state_;
  CalculatorOptions options_;
};

TEST_F(DistanceCalculatorTest, SeparatedSpheresHaveAnalyticClearance)
{
  addSphere("obstacle", Eigen::Vector3d(0.35, 0.0, 0.0));
  expectMinimum(compute(), 0.15);
}

TEST_F(DistanceCalculatorTest, SphereToBoxHasAnalyticClearance)
{
  addBox("obstacle", Eigen::Vector3d(0.4, 0.0, 0.0));
  expectMinimum(compute(), 0.2);
}

TEST_F(DistanceCalculatorTest, TinyPositiveGapIsNotCollision)
{
  addSphere("obstacle", Eigen::Vector3d(0.201, 0.0, 0.0));
  const auto summary = compute();
  expectMinimum(summary, 0.001);
  EXPECT_GT(summary.overall_min_distance, 0.0);
}

TEST_F(DistanceCalculatorTest, TouchingSpheresHaveZeroDistance)
{
  addSphere("obstacle", Eigen::Vector3d(0.2, 0.0, 0.0));
  expectMinimum(compute(), 0.0);
}

TEST_F(DistanceCalculatorTest, OverlappingSpheresHavePhysicalPenetrationDepth)
{
  addSphere("obstacle", Eigen::Vector3d(0.15, 0.0, 0.0));
  expectMinimum(compute(), -0.05);
}

TEST_F(DistanceCalculatorTest, OverlappingSphereAndBoxHavePhysicalPenetrationDepth)
{
  addBox("obstacle", Eigen::Vector3d(0.15, 0.0, 0.0));
  expectMinimum(compute(), -0.05);
}

TEST_F(DistanceCalculatorTest, ChoosesClosestPairAndMatchingNames)
{
  addSphere("far", Eigen::Vector3d(0.6, 0.0, 0.0));
  addSphere("near", Eigen::Vector3d(0.3, 0.0, 0.0));
  expectMinimum(compute(), 0.1, "base_link", "near");
}

TEST_F(DistanceCalculatorTest, ChoosesDeepestOverlapAcrossObstacles)
{
  addSphere("shallow", Eigen::Vector3d(0.18, 0.0, 0.0));
  addSphere("deep", Eigen::Vector3d(0.0, 0.15, 0.0));
  expectMinimum(compute(), -0.05, "base_link", "deep");
}

TEST_F(DistanceCalculatorTest, EmptyWorldHasNoFiniteDistance)
{
  expectNoPairs(compute());
}

TEST_F(DistanceCalculatorTest, ObstaclesBeyondThresholdHaveNoReportedPairs)
{
  addSphere("obstacle", Eigen::Vector3d(0.8, 0.0, 0.0));
  options_.distance_threshold = 0.5;
  expectNoPairs(compute());
  options_.distance_threshold = 1.0;
  expectMinimum(compute(), 0.6);
}

TEST_F(DistanceCalculatorTest, AllowedCollisionPairIsExcluded)
{
  addSphere("ignored", Eigen::Vector3d(0.15, 0.0, 0.0));
  addSphere("obstacle", Eigen::Vector3d(0.4, 0.0, 0.0));
  scene_->getAllowedCollisionMatrixNonConst().setEntry("base_link", "ignored", true);
  const auto summary = compute();
  expectMinimum(summary, 0.2);
  for (const auto & item : summary.items) {
    EXPECT_NE(item.obstacle_id, "ignored");
  }
}

TEST_F(DistanceCalculatorTest, GroupFilterExcludesCollisionOnOtherLinks)
{
  addSphere("base_obstacle", Eigen::Vector3d(0.15, 0.0, 0.0));
  addSphere("tip_obstacle", Eigen::Vector3d(0.4, 2.0, 0.0));
  options_.group_name = "tip_only";
  expectMinimum(compute(), 0.2, "tip_link", "tip_obstacle");
}

TEST_F(DistanceCalculatorTest, UsesSuppliedRobotStateRatherThanSceneDefault)
{
  addSphere("obstacle", Eigen::Vector3d(0.4, 0.0, 0.0));
  expectMinimum(compute(), 0.2);
  moveRobot(Eigen::Vector3d(0.1, 0.0, 0.0));
  expectMinimum(compute(), 0.1);
}

TEST_F(DistanceCalculatorTest, NearestPointsAndNormalAreInWorldFrame)
{
  const Eigen::Vector3d translation(1.0, 2.0, 3.0);
  moveRobot(translation, std::acos(-1.0) / 2.0);
  addBox("obstacle", translation + Eigen::Vector3d(0.0, 0.4, 0.0));
  const auto summary = compute();
  expectMinimum(summary, 0.2);
  ASSERT_EQ(summary.items.size(), 1u);
  const auto & item = summary.items.front();
  EXPECT_NEAR((item.point_on_link - (translation + Eigen::Vector3d(0.0, 0.1, 0.0))).norm(), 0.0, 1e-4);
  EXPECT_NEAR((item.point_on_obstacle - (translation + Eigen::Vector3d(0.0, 0.3, 0.0))).norm(), 0.0, 1e-4);
  EXPECT_NEAR((item.normal - Eigen::Vector3d::UnitY()).norm(), 0.0, 1e-4);
  EXPECT_NEAR((item.point_on_obstacle - item.point_on_link).norm(), item.distance, 1e-5);
}

TEST_F(DistanceCalculatorTest, PaddingChoiceChangesClearanceByConfiguredPadding)
{
  addSphere("obstacle", Eigen::Vector3d(0.4, 0.0, 0.0));
  scene_->getCollisionEnvNonConst()->setLinkPadding("base_link", 0.02);
  options_.use_unpadded_env = true;
  expectMinimum(compute(), 0.2);
  options_.use_unpadded_env = false;
  expectMinimum(compute(), 0.18);
}

TEST_F(DistanceCalculatorTest, MeshSphereClearanceAndNearestPointsUseWorldFrame)
{
  const Eigen::Vector3d translation(1.0, 2.0, 3.0);
  const double yaw = std::acos(-1.0) / 2.0;
  moveRobot(translation, yaw);
  addMeshBox("obstacle", translation + Eigen::Vector3d(0.0, 0.3, 0.0), yaw);
  const auto summary = compute();
  expectMinimum(summary, 0.1);
  ASSERT_EQ(summary.items.size(), 1u);
  const auto & item = summary.items.front();
  EXPECT_NEAR((item.point_on_link - (translation + Eigen::Vector3d(0.0, 0.1, 0.0))).norm(), 0.0, 1e-4);
  EXPECT_NEAR((item.point_on_obstacle - (translation + Eigen::Vector3d(0.0, 0.2, 0.0))).norm(), 0.0, 1e-4);
  EXPECT_NEAR((item.normal - Eigen::Vector3d::UnitY()).norm(), 0.0, 1e-4);
  EXPECT_NEAR((item.point_on_obstacle - item.point_on_link).norm(), item.distance, 1e-5);
}

TEST_F(DistanceCalculatorTest, MeshSphereOverlapHasPhysicalDepthAtNonidentityPose)
{
  const Eigen::Vector3d translation(1.0, 2.0, 3.0);
  const double yaw = std::acos(-1.0) / 2.0;
  moveRobot(translation, yaw);
  addMeshBox("obstacle", translation + Eigen::Vector3d(0.0, 0.15, 0.0), yaw);
  expectMinimum(compute(), -0.05);
}

TEST_F(DistanceCalculatorTest, MeshSphereTinyGapRemainsPositive)
{
  addMeshBox("obstacle", Eigen::Vector3d(0.201, 0.0, 0.0));
  const auto summary = compute();
  expectMinimum(summary, 0.001);
  EXPECT_GT(summary.overall_min_distance, 0.0);
}

TEST_F(DistanceCalculatorTest, MeshSphereContactIsZero)
{
  addMeshBox("obstacle", Eigen::Vector3d(0.2, 0.0, 0.0));
  expectMinimum(compute(), 0.0);
}

TEST_F(DistanceCalculatorTest, SphereCenterInsideClosedMeshIsNegative)
{
  addMeshBox("obstacle", Eigen::Vector3d(0.02, 0.0, 0.0));
  expectMinimum(compute(), -0.18);
}

TEST_F(DistanceCalculatorTest, MeshSphereSweepCrossesZeroContinuously)
{
  // Changing the robot pose must change the measured clearance on every call.
  addMeshBox("obstacle", Eigen::Vector3d(0.3, 0.0, 0.0));
  for (double x : {0.0, 0.08, 0.099, 0.1, 0.101, 0.12, 0.15}) {
    moveRobot(Eigen::Vector3d(x, 0.0, 0.0));
    expectMinimum(compute(), 0.1 - x);
  }
}

TEST_F(DistanceCalculatorTest, RobotAttachedMeshToWorldSphereHasMatchingWitnesses)
{
  addMeshBox("mesh_template", Eigen::Vector3d::Zero());
  const auto mesh = scene_->getWorld()->getObject("mesh_template")->shapes_.front();
  scene_->getWorldNonConst()->removeObject("mesh_template");
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation().z() = 0.5;
  state_->attachBody("tool_mesh", Eigen::Isometry3d::Identity(), {mesh}, {offset},
    std::set<std::string>{"base_link"}, "base_link");
  const Eigen::Vector3d translation(1.0, 2.0, 3.0);
  moveRobot(translation, 0.7);
  addSphere("obstacle", translation + Eigen::Vector3d(0.0, 0.0, 0.8));
  const auto summary = compute();
  expectMinimum(summary, 0.1, "tool_mesh", "obstacle");
  const auto & item = summary.items.front();
  EXPECT_NEAR((item.point_on_link - (translation + Eigen::Vector3d(0.0, 0.0, 0.6))).norm(), 0.0, 1e-8);
  EXPECT_NEAR((item.point_on_obstacle - (translation + Eigen::Vector3d(0.0, 0.0, 0.7))).norm(), 0.0, 1e-8);
  EXPECT_NEAR((item.normal - Eigen::Vector3d::UnitZ()).norm(), 0.0, 1e-8);
}

TEST_F(DistanceCalculatorTest, ObjectAndShapePosesAreBothApplied)
{
  Eigen::Isometry3d object_pose = Eigen::Isometry3d::Identity();
  object_pose.translation().y() = 1.0;
  Eigen::Isometry3d shape_pose = Eigen::Isometry3d::Identity();
  shape_pose.translation().y() = -0.7;
  scene_->getWorldNonConst()->addToObject("obstacle", object_pose,
    std::make_shared<shapes::Sphere>(0.1), shape_pose);
  expectMinimum(compute(), 0.1);
}

}  // namespace
}  // namespace obstacle_distance_calculator
