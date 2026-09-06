#pragma once

#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/shape/sphere.h>
#include <moveit/collision_detection_fcl/collision_common.h>

namespace obstacle_distance_calculator
{
namespace detail
{
inline Eigen::Vector3d closestOnSegment(
  const Eigen::Vector3d & p, const Eigen::Vector3d & a, const Eigen::Vector3d & b)
{
  const Eigen::Vector3d edge = b - a;
  const double t = edge.squaredNorm() > 0.0 ?
    std::clamp((p - a).dot(edge) / edge.squaredNorm(), 0.0, 1.0) : 0.0;
  return a + t * edge;
}

inline Eigen::Vector3d closestOnTriangle(
  const Eigen::Vector3d & p, const Eigen::Vector3d & a,
  const Eigen::Vector3d & b, const Eigen::Vector3d & c)
{
  const Eigen::Vector3d normal = (b - a).cross(c - a);
  if (normal.squaredNorm() > 0.0) {
    const Eigen::Vector3d projected = p - normal * ((p - a).dot(normal) / normal.squaredNorm());
    if ((b - a).cross(projected - a).dot(normal) >= 0.0 &&
        (c - b).cross(projected - b).dot(normal) >= 0.0 &&
        (a - c).cross(projected - c).dot(normal) >= 0.0) {
      return projected;
    }
  }
  // Also handles zero-area triangles as line segments.
  Eigen::Vector3d closest = closestOnSegment(p, a, b);
  for (const auto & candidate : {closestOnSegment(p, b, c), closestOnSegment(p, c, a)}) {
    if ((candidate - p).squaredNorm() < (closest - p).squaredNorm()) {
      closest = candidate;
    }
  }
  return closest;
}

// Closed meshes: count ray/surface crossings, merging shared-edge hits.
inline bool insideMesh(const fcl::BVHModel<fcl::OBBRSSd> & mesh, const Eigen::Vector3d & p)
{
  if ((p.array() < mesh.aabb_local.min_.array()).any() ||
      (p.array() > mesh.aabb_local.max_.array()).any()) {
    return false;
  }
  const Eigen::Vector3d ray = Eigen::Vector3d(1.0, 0.371390676, 0.529128421).normalized();
  std::vector<double> intersections;
  for (int i = 0; i < mesh.num_tris; ++i) {
    const auto & triangle = mesh.tri_indices[i];
    const Eigen::Vector3d & a = mesh.vertices[triangle[0]];
    const Eigen::Vector3d edge1 = mesh.vertices[triangle[1]] - a;
    const Eigen::Vector3d edge2 = mesh.vertices[triangle[2]] - a;
    const Eigen::Vector3d h = ray.cross(edge2);
    const double determinant = edge1.dot(h);
    if (std::abs(determinant) <= 1e-12 * edge1.norm() * edge2.norm()) {
      continue;
    }
    const Eigen::Vector3d offset = p - a;
    const double u = offset.dot(h) / determinant;
    const Eigen::Vector3d q = offset.cross(edge1);
    const double v = ray.dot(q) / determinant;
    const double t = edge2.dot(q) / determinant;
    if (u >= 0.0 && v >= 0.0 && u + v <= 1.0 && t > 0.0) {
      intersections.push_back(t);
    }
  }
  std::sort(intersections.begin(), intersections.end());
  const auto end = std::unique(intersections.begin(), intersections.end(),
    [](double a, double b) {return std::abs(a - b) < 1e-9;});
  return std::distance(intersections.begin(), end) % 2 == 1;
}

// Bypass FCL 0.7's sphereTriangleDistance: it leaves outputs uninitialized
// on overlap and supplies local-frame witnesses for separated mesh/sphere pairs.
inline bool meshSphereDistance(
  const fcl::CollisionObjectd & first, const fcl::CollisionObjectd & second,
  collision_detection::DistanceResultsData & result)
{
  const bool mesh_first = first.getNodeType() == fcl::BV_OBBRSS && second.getNodeType() == fcl::GEOM_SPHERE;
  const bool mesh_second = second.getNodeType() == fcl::BV_OBBRSS && first.getNodeType() == fcl::GEOM_SPHERE;
  if (!mesh_first && !mesh_second) {
    return false;
  }
  const auto & mesh_object = mesh_first ? first : second;
  const auto & sphere_object = mesh_first ? second : first;
  const auto & mesh = static_cast<const fcl::BVHModel<fcl::OBBRSSd> &>(*mesh_object.collisionGeometry());
  const auto & sphere = static_cast<const fcl::Sphered &>(*sphere_object.collisionGeometry());
  const Eigen::Vector3d center = mesh_object.getTransform().inverse() * sphere_object.getTranslation();
  double squared_distance = std::numeric_limits<double>::infinity();
  Eigen::Vector3d closest = Eigen::Vector3d::Zero();
  Eigen::Vector3d surface_normal = Eigen::Vector3d::UnitX();
  // Reuse FCL's bounding-volume tree, visiting nearby triangles first. A
  // subtree farther than the best point found cannot contain a closer point.
  const auto lowerBound = [&mesh, &center](int index) {
    const auto & box = mesh.getBV(index).bv.obb;
    const Eigen::Vector3d offset = box.axis.transpose() * (center - box.To);
    return (offset.cwiseAbs() - box.extent).cwiseMax(0.0).squaredNorm();
  };
  using Candidate = std::pair<double, int>;
  std::priority_queue<Candidate, std::vector<Candidate>, std::greater<Candidate>> pending;
  if (mesh.num_tris > 0) {
    pending.emplace(lowerBound(0), 0);
  }
  while (!pending.empty()) {
    const auto candidate_node = pending.top();
    pending.pop();
    if (candidate_node.first > squared_distance + 1e-14) {
      continue;
    }
    const auto & node = mesh.getBV(candidate_node.second);
    if (!node.isLeaf()) {
      pending.emplace(lowerBound(node.leftChild()), node.leftChild());
      pending.emplace(lowerBound(node.rightChild()), node.rightChild());
      continue;
    }
    const auto & triangle = mesh.tri_indices[node.primitiveId()];
    const auto & a = mesh.vertices[triangle[0]];
    const auto & b = mesh.vertices[triangle[1]];
    const auto & c = mesh.vertices[triangle[2]];
    const Eigen::Vector3d candidate = closestOnTriangle(center, a, b, c);
    const double candidate_squared_distance = (center - candidate).squaredNorm();
    if (candidate_squared_distance < squared_distance) {
      squared_distance = candidate_squared_distance;
      closest = candidate;
      const Eigen::Vector3d normal = (b - a).cross(c - a);
      if (normal.norm() > 0.0) {
        surface_normal = normal.normalized();
      }
    }
  }
  const double center_distance = std::sqrt(squared_distance);
  const bool inside = center_distance > 1e-12 && insideMesh(mesh, center);
  Eigen::Vector3d direction = surface_normal;
  if (center_distance > 1e-12) {
    direction = (center - closest) / center_distance;
  }
  if (inside) {
    direction = -direction;
  }
  const Eigen::Vector3d world_direction = mesh_object.getRotation() * direction;
  result.distance = (inside ? -center_distance : center_distance) - sphere.radius;
  const int mesh_index = mesh_first ? 0 : 1;
  result.nearest_points[mesh_index] = mesh_object.getTransform() * closest;
  result.nearest_points[1 - mesh_index] = sphere_object.getTranslation() - sphere.radius * world_direction;
  result.normal = mesh_first ? world_direction : -world_direction;
  const fcl::CollisionObjectd * objects[] = {&first, &second};
  for (int i = 0; i < 2; ++i) {
    const auto * data = static_cast<const collision_detection::CollisionGeometryData *>(
      objects[i]->collisionGeometry()->getUserData());
    result.link_names[i] = data->getID();
    result.body_types[i] = data->type;
  }
  return true;
}
}  // namespace detail
}  // namespace obstacle_distance_calculator
