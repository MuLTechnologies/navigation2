// Copyright (c) 2020, Samsung Research America
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

#include <vector>
#include <memory>
#include <utility>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_smac_planner/constants.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#ifndef NAV2_SMAC_PLANNER__COLLISION_CHECKER_HPP_
#define NAV2_SMAC_PLANNER__COLLISION_CHECKER_HPP_

namespace nav2_smac_planner
{

/**
 * @class nav2_smac_planner::GridCollisionChecker
 * @brief A costmap grid collision checker
 */
class GridCollisionChecker
  : public nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
{
public:
  /**
   * @brief A constructor for nav2_smac_planner::GridCollisionChecker
   * for use when regular bin intervals are appropriate
   * @param costmap The costmap to collision check against
   * @param num_quantizations The number of quantizations to precompute footprint
   * @param node Node to extract clock and logger from
   * orientations for to speed up collision checking
   */
  GridCollisionChecker(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap,
    unsigned int num_quantizations,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  /**
   * @brief A constructor for nav2_smac_planner::GridCollisionChecker
   * for use when irregular bin intervals are appropriate
   * @param costmap The costmap to collision check against
   * @param angles The vector of possible angle bins to precompute for
   * orientations for to speed up collision checking, in radians
   */
  // GridCollisionChecker(
  //   nav2_costmap_2d::Costmap2D * costmap,
  //   std::vector<float> & angles);

  /**
   * @brief Set the footprint to use with collision checker
   * @param footprint The footprint to collision check against
   * @param radius Whether or not the footprint is a circle and use radius collision checking
   */
  void setFootprint(
    const nav2_costmap_2d::Footprint & footprint,
    const bool & radius,
    const double & possible_collision_cost);

  /**
   * @brief Check if in collision with costmap and footprint at pose
   * @param x X coordinate of pose to check against
   * @param y Y coordinate of pose to check against
   * @param theta Angle bin number of pose to check against (NOT radians)
   * @param traverse_unknown Whether or not to traverse in unknown space
   * @return boolean if in collision or not.
   */
  bool inCollision(
    const float & x,
    const float & y,
    const float & theta,
    const bool & traverse_unknown);

  /**
   * @brief Check if in collision with costmap and footprint at pose
   * @param i Index to search collision status of
   * @param traverse_unknown Whether or not to traverse in unknown space
   * @return boolean if in collision or not.
   */
  bool inCollision(
    const unsigned int & i,
    const bool & traverse_unknown);

  /**
   * @brief Get cost at footprint pose in costmap
   * @return the cost at the pose in costmap
   */
  float getCost();

  /**
   * @brief Max cost along the full footprint outline from the last pose inCollision() checked
   * @return the outline cost, or -1 if that call didn't rasterize the footprint
   * (far from obstacles, radius footprints, or an early collision return)
   */
  float getFootprintCost() const;

  /**
   * @brief Score a pose by sampling cost at the footprint's vertices, edge midpoints
   * and centroid. Read-only: does not affect collision results or getCost().
   * @param x X coordinate of pose, in cells
   * @param y Y coordinate of pose, in cells
   * @param angle_bin Angle bin number of pose (NOT radians)
   * @param use_max Return the highest sampled cost instead of the mean
   * @return the score, in costmap cost units
   */
  float scoreFootprint(
    const float & x,
    const float & y,
    const float & angle_bin,
    const bool & use_max);

  /**
   * @brief Get the angles of the precomputed footprint orientations
   * @return the ordered vector of angles corresponding to footprints
   */
  std::vector<float> & getPrecomputedAngles()
  {
    return angles_;
  }

  /**
   * @brief Get costmap ros object for inflation layer params
   * @return Costmap ros
   */
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> getCostmapROS() {return costmap_ros_;}

  /**
   * @brief Check if value outside the range
   * @param min Minimum value of the range
   * @param max Maximum value of the range
   * @param value the value to check if it is within the range
   * @return boolean if in range or not
   */
  bool outsideRange(const unsigned int & max, const float & value);

protected:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::vector<nav2_costmap_2d::Footprint> oriented_footprints_;
  nav2_costmap_2d::Footprint unoriented_footprint_;
  // Score sample points per angle bin, as offsets in cells from the pose
  std::vector<std::vector<std::pair<float, float>>> oriented_score_points_;
  double score_points_resolution_{0.0};
  float center_cost_;
  float footprint_cost_{-1.0f};
  bool footprint_is_radius_{false};
  std::vector<float> angles_;
  float possible_collision_cost_{-1};
  rclcpp::Logger logger_{rclcpp::get_logger("SmacPlannerCollisionChecker")};
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__COLLISION_CHECKER_HPP_
