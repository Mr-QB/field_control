#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "obstacle_distance_calculator/distance_calculator.hpp"

namespace obstacle_distance_calculator
{
struct Sample { std::vector<double> q; double clearance; };

class FieldDatasetGenerator : public rclcpp::Node
{
public:
  FieldDatasetGenerator() : Node("field_dataset_generator")
  {
    robot_description_ = declare_parameter("robot_description_parameter", "robot_description");
    group_name_ = declare_parameter("group_name", "ur_manipulator");
    output_csv_ = declare_parameter("output_csv", "field_dataset.csv");
    metadata_json_ = declare_parameter("metadata_json", "field_dataset_metadata.json");
    samples_ = declare_parameter("samples", 10000);
    candidate_multiplier_ = declare_parameter("candidate_multiplier", 10);
    seed_ = declare_parameter("seed", 42);
    clearance_cap_ = declare_parameter("clearance_cap", 1.0);
  }

  void init()
  {
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      shared_from_this(), robot_description_, "field_dataset_monitor");
    psm_->startStateMonitor();
    psm_->startWorldGeometryMonitor();
    psm_->startSceneMonitor("/monitored_planning_scene");
    psm_->requestPlanningSceneState();
    timer_ = create_wall_timer(std::chrono::seconds(2), std::bind(&FieldDatasetGenerator::generate, this));
  }

private:
  void generate()
  {
    timer_->cancel();
    planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
    const auto * group = scene->getRobotModel()->getJointModelGroup(group_name_);
    if (!group) {
      RCLCPP_ERROR(get_logger(), "Planning group '%s' does not exist", group_name_.c_str());
      rclcpp::shutdown();
      return;
    }
    const auto & names = group->getVariableNames();
    std::vector<moveit::core::VariableBounds> bounds;
    for (const auto * joint_bounds : group->getActiveJointModelsBounds()) {
      bounds.insert(bounds.end(), joint_bounds->begin(), joint_bounds->end());
    }
    if (names.empty() || names.size() != bounds.size() || samples_ <= 0 || clearance_cap_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "Invalid group bounds or generator parameters");
      rclcpp::shutdown();
      return;
    }

    std::mt19937 rng(seed_);
    std::vector<Sample> bins[5];
    CalculatorOptions options;
    // q contains every active variable of this group. Leave the request ungroupped:
    // MoveIt Humble's FCL grouped distance map may include non-distance entries.
    options.group_name = "";
    options.distance_threshold = clearance_cap_;
    options.use_unpadded_env = true;
    const auto candidates = samples_ * std::max(1, candidate_multiplier_);
    for (int n = 0; n < candidates; ++n) {
      moveit::core::RobotState state(scene->getCurrentState());
      std::vector<double> q;
      q.reserve(bounds.size());
      for (const auto & bound : bounds) {
        q.push_back(std::uniform_real_distribution<double>(bound.min_position_, bound.max_position_)(rng));
      }
      state.setJointGroupPositions(group, q);
      state.update();
      auto summary = DistanceCalculator::compute(scene, state, options);
      const double d = std::isfinite(summary.overall_min_distance) ?
        summary.overall_min_distance : clearance_cap_;
      bins[bin(d)].push_back({q, std::min(d, clearance_cap_)});
    }

    std::vector<Sample> selected;
    const int quota = std::max(1, samples_ / 5);
    for (auto & bucket : bins) {
      std::shuffle(bucket.begin(), bucket.end(), rng);
      const auto take = std::min<int>(quota, bucket.size());
      selected.insert(selected.end(), bucket.begin(), bucket.begin() + take);
    }
    for (auto & bucket : bins) {
      for (const auto & sample : bucket) {
        if (static_cast<int>(selected.size()) >= samples_) break;
        selected.push_back(sample);
      }
      if (static_cast<int>(selected.size()) >= samples_) break;
    }
    write_csv(selected, names);
    write_metadata(names, bounds, scene->getPlanningFrame());
    RCLCPP_INFO(get_logger(), "Wrote %zu samples to %s", selected.size(), output_csv_.c_str());
    RCLCPP_INFO(get_logger(), "Histogram collision=%zu very_near=%zu near=%zu medium=%zu far=%zu",
      bins[0].size(), bins[1].size(), bins[2].size(), bins[3].size(), bins[4].size());
    rclcpp::shutdown();
  }

  static int bin(double d)
  {
    if (d <= 0.0) return 0;
    if (d < 0.05) return 1;
    if (d < 0.15) return 2;
    if (d < 0.30) return 3;
    return 4;
  }

  void write_csv(const std::vector<Sample> & data, const std::vector<std::string> & names) const
  {
    std::ofstream out(output_csv_);
    for (const auto & name : names) out << name << ',';
    out << "clearance,is_collision\n" << std::setprecision(10);
    for (const auto & sample : data) {
      for (double value : sample.q) out << value << ',';
      out << sample.clearance << ',' << (sample.clearance <= 0.0 ? 1 : 0) << '\n';
    }
  }

  void write_metadata(const std::vector<std::string> & names,
    const std::vector<moveit::core::VariableBounds> & bounds, const std::string & frame) const
  {
    std::ofstream out(metadata_json_);
    out << std::setprecision(10) << "{\n  \"joint_names\": [";
    for (size_t i = 0; i < names.size(); ++i) out << (i ? ", " : "") << '\"' << names[i] << '\"';
    out << "],\n  \"lower_bounds\": [";
    for (size_t i = 0; i < bounds.size(); ++i) out << (i ? ", " : "") << bounds[i].min_position_;
    out << "],\n  \"upper_bounds\": [";
    for (size_t i = 0; i < bounds.size(); ++i) out << (i ? ", " : "") << bounds[i].max_position_;
    out << "],\n  \"group_name\": \"" << group_name_ << "\",\n  \"number_of_samples\": " << samples_
        << ",\n  \"seed\": " << seed_ << ",\n  \"clearance_cap\": " << clearance_cap_
        << ",\n  \"planning_frame\": \"" << frame << "\"\n}\n";
  }

  std::string robot_description_, group_name_, output_csv_, metadata_json_;
  int samples_, candidate_multiplier_, seed_;
  double clearance_cap_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace obstacle_distance_calculator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<obstacle_distance_calculator::FieldDatasetGenerator>();
  node->init();
  rclcpp::spin(node);
  return 0;
}
