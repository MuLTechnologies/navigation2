// Copyright (c) 2021, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <algorithm>

#include "nav2_smac_planner/collision_checker.hpp"

namespace nav2_smac_planner
{

GridCollisionChecker::GridCollisionChecker(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  unsigned int num_quantizations,
  rclcpp_lifecycle::LifecycleNode::SharedPtr node)
: FootprintCollisionChecker(costmap_ros ? costmap_ros->getCostmap() : nullptr)
{
  if (node) {
    clock_ = node->get_clock();
    logger_ = node->get_logger();
  }

  if (costmap_ros) {
    costmap_ros_ = costmap_ros;
  }

  // Convert number of regular bins into angles
  float bin_size = 2 * M_PI / static_cast<float>(num_quantizations);
  angles_.reserve(num_quantizations);
  for (unsigned int i = 0; i != num_quantizations; i++) {
    angles_.push_back(bin_size * i);
  }
}

// GridCollisionChecker::GridCollisionChecker(
//   nav2_costmap_2d::Costmap2D * costmap,
//   std::vector<float> & angles)
// : FootprintCollisionChecker(costmap),
//   angles_(angles)
// {
// }

void GridCollisionChecker::setFootprint(
  const nav2_costmap_2d::Footprint & footprint,
  const bool & radius,
  const double & possible_collision_cost)
{
  possible_collision_cost_ = static_cast<float>(possible_collision_cost);
  if (possible_collision_cost_ <= 0.0f && !radius) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 1000,
      "Inflation layer either not found or inflation is not set sufficiently for "
      "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
      " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
      "github.com/ros-planning/navigation2/tree/main/nav2_smac_planner#potential-fields"
      " for full instructions. This will substantially impact run-time performance.");
  }

  footprint_is_radius_ = radius;

  // Use radius, no caching required
  if (radius) {
    return;
  }

  // No change, no updates required
  const double resolution = costmap_->getResolution();
  if (footprint == unoriented_footprint_ && resolution == score_points_resolution_) {
    return;
  }

  oriented_footprints_.clear();
  oriented_footprints_.reserve(angles_.size());
  double sin_th, cos_th;
  geometry_msgs::msg::Point new_pt;
  const unsigned int footprint_size = footprint.size();

  // Score sample points: vertices, edge midpoints and the vertex centroid
  std::vector<std::pair<double, double>> score_points;
  score_points.reserve(2 * footprint_size + 1);
  double cx = 0.0, cy = 0.0;
  for (unsigned int j = 0; j < footprint_size; j++) {
    const auto & a = footprint[j];
    const auto & b = footprint[(j + 1) % footprint_size];
    score_points.emplace_back(a.x, a.y);
    score_points.emplace_back(0.5 * (a.x + b.x), 0.5 * (a.y + b.y));
    cx += a.x;
    cy += a.y;
  }
  if (footprint_size > 0) {
    score_points.emplace_back(cx / footprint_size, cy / footprint_size);
  }
  oriented_score_points_.clear();
  oriented_score_points_.reserve(angles_.size());

  // Precompute the orientation bins for checking to use
  for (unsigned int i = 0; i != angles_.size(); i++) {
    sin_th = sin(angles_[i]);
    cos_th = cos(angles_[i]);
    nav2_costmap_2d::Footprint oriented_footprint;
    oriented_footprint.reserve(footprint_size);

    for (unsigned int j = 0; j < footprint_size; j++) {
      new_pt.x = footprint[j].x * cos_th - footprint[j].y * sin_th;
      new_pt.y = footprint[j].x * sin_th + footprint[j].y * cos_th;
      oriented_footprint.push_back(new_pt);
    }

    oriented_footprints_.push_back(oriented_footprint);

    std::vector<std::pair<float, float>> oriented_points;
    oriented_points.reserve(score_points.size());
    for (const auto & p : score_points) {
      oriented_points.emplace_back(
        static_cast<float>((p.first * cos_th - p.second * sin_th) / resolution),
        static_cast<float>((p.first * sin_th + p.second * cos_th) / resolution));
    }
    oriented_score_points_.push_back(oriented_points);
  }

  unoriented_footprint_ = footprint;
  score_points_resolution_ = resolution;
}

bool GridCollisionChecker::inCollision(
  const float & x,
  const float & y,
  const float & angle_bin,
  const bool & traverse_unknown)
{
  footprint_cost_ = -1.0f;

  // Check to make sure cell is inside the map
  if (outsideRange(costmap_->getSizeInCellsX(), x) ||
    outsideRange(costmap_->getSizeInCellsY(), y))
  {
    return true;
  }

  // Assumes setFootprint already set
  center_cost_ = static_cast<float>(costmap_->getCost(
    static_cast<unsigned int>(x + 0.5f), static_cast<unsigned int>(y + 0.5f)));

  if (!footprint_is_radius_) {
    // if footprint, then we check for the footprint's points, but first see
    // if the robot is even potentially in an inscribed collision
    if (center_cost_ < possible_collision_cost_ && possible_collision_cost_ > 0.0f) {
      return false;
    }

    // If its inscribed, in collision, or unknown in the middle,
    // no need to even check the footprint, its invalid
    if (center_cost_ == UNKNOWN_COST && !traverse_unknown) {
      return true;
    }

    if (center_cost_ == INSCRIBED_COST || center_cost_ == OCCUPIED_COST) {
      return true;
    }

    // if possible inscribed, need to check actual footprint pose.
    // Use precomputed oriented footprints are done on initialization,
    // offset by translation value to collision check
    double wx, wy;
    costmap_->mapToWorld(static_cast<double>(x), static_cast<double>(y), wx, wy);
    geometry_msgs::msg::Point new_pt;
    // Wrap like scoreFootprint(): a primitive pose at exactly 2*pi gives bin == size
    const nav2_costmap_2d::Footprint & oriented_footprint =
      oriented_footprints_[static_cast<unsigned int>(angle_bin) % oriented_footprints_.size()];
    nav2_costmap_2d::Footprint current_footprint;
    current_footprint.reserve(oriented_footprint.size());
    for (unsigned int i = 0; i < oriented_footprint.size(); ++i) {
      new_pt.x = wx + oriented_footprint[i].x;
      new_pt.y = wy + oriented_footprint[i].y;
      current_footprint.push_back(new_pt);
    }

    float footprint_cost = static_cast<float>(footprintCost(current_footprint));
    footprint_cost_ = footprint_cost;

    if (footprint_cost == UNKNOWN_COST && traverse_unknown) {
      return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return footprint_cost >= OCCUPIED_COST;
  } else {
    // if radius, then we can check the center of the cost assuming inflation is used
    if (center_cost_ == UNKNOWN_COST && traverse_unknown) {
      return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return center_cost_ >= INSCRIBED_COST;
  }
}

bool GridCollisionChecker::inCollision(
  const unsigned int & i,
  const bool & traverse_unknown)
{
  footprint_cost_ = -1.0f;
  center_cost_ = costmap_->getCost(i);
  if (center_cost_ == UNKNOWN_COST && traverse_unknown) {
    return false;
  }

  // if occupied or unknown and not to traverse unknown space
  return center_cost_ >= INSCRIBED_COST;
}

float GridCollisionChecker::getCost()
{
  // Assumes inCollision called prior
  return static_cast<float>(center_cost_);
}

float GridCollisionChecker::getFootprintCost() const
{
  return footprint_cost_;
}

float GridCollisionChecker::scoreFootprint(
  const float & x,
  const float & y,
  const float & angle_bin,
  const bool & use_max)
{
  const unsigned int size_x = costmap_->getSizeInCellsX();
  const unsigned int size_y = costmap_->getSizeInCellsY();
  // Off-map samples score as occupied; collision checking rejects those poses separately
  auto cellCost = [&](const float & fx, const float & fy) {
      if (fx < 0.0f || fy < 0.0f || fx >= size_x || fy >= size_y) {
        return OCCUPIED_COST;
      }
      return static_cast<float>(costmap_->getCost(
               static_cast<unsigned int>(fx), static_cast<unsigned int>(fy)));
    };

  if (footprint_is_radius_ || oriented_score_points_.empty() ||
    oriented_score_points_[0].empty())
  {
    return cellCost(x + 0.5f, y + 0.5f);
  }

  const auto & points =
    oriented_score_points_[static_cast<unsigned int>(angle_bin) % oriented_score_points_.size()];
  float sum = 0.0f;
  float max = 0.0f;
  for (const auto & p : points) {
    const float cost = cellCost(x + p.first + 0.5f, y + p.second + 0.5f);
    sum += cost;
    max = std::max(max, cost);
  }
  return use_max ? max : sum / static_cast<float>(points.size());
}

bool GridCollisionChecker::outsideRange(const unsigned int & max, const float & value)
{
  return value < 0.0f || value > max;
}

}  // namespace nav2_smac_planner
