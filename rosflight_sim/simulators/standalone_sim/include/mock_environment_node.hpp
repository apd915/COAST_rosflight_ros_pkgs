/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2025
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MOCK_ENVIRONMENT_NODE_HPP
#define MOCK_ENVIRONMENT_NODE_HPP

#include <vector>
#include <cmath>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include "rosflight_msgs/msg/sim_state.hpp"

namespace rosflight_sim
{

// ---------------------------------------------------------------------------
// Obstacle primitives -- each knows how to intersect a ray
// ---------------------------------------------------------------------------

struct AABB
{
  Eigen::Vector3d min_corner;  // NED metres
  Eigen::Vector3d max_corner;

  /// Ray–AABB intersection (slab method).  Returns t >= 0 on hit, or -1.
  double intersect(const Eigen::Vector3d & origin,
                   const Eigen::Vector3d & dir) const;
};

struct Cylinder
{
  Eigen::Vector3d center;  // NED metres (center of the cylinder axis)
  double radius;
  double half_height;      // extends ±half_height along the D (z-NED) axis

  /// Ray–cylinder intersection.  Returns t >= 0 on hit, or -1.
  double intersect(const Eigen::Vector3d & origin,
                   const Eigen::Vector3d & dir) const;
};

// ---------------------------------------------------------------------------
// The ROS 2 node
// ---------------------------------------------------------------------------

class MockEnvironmentNode : public rclcpp::Node
{
public:
  MockEnvironmentNode();

private:
  // --- Parameters ---
  void declare_parameters();

  // --- Obstacle definitions ---
  void build_obstacle_world();

  // --- Callbacks ---
  void state_callback(const rosflight_msgs::msg::SimState & msg);
  void lidar_timer_callback();

  // --- LiDAR raycasting ---
  /// Cast a single ray and return hit distance (or max_range if no hit).
  double cast_ray(const Eigen::Vector3d & origin,
                  const Eigen::Vector3d & direction,
                  double max_range) const;

  /// Build and publish the full PointCloud2 scan.
  void publish_lidar_scan();

  // --- RViz marker helpers ---
  void publish_obstacle_markers();

  /// Check the TRUE vehicle position against the obstacle set and report contact.
  ///
  /// The sim has no physics, so a vehicle that flies into a trunk passes straight through and
  /// nothing records it -- which makes "reached the goal" indistinguishable from "reached the
  /// goal without collision", and the latter is what a success rate has to mean. This closes
  /// that gap using the same exact primitives the raycaster already uses.
  void check_collision();

  // --- ROS interfaces ---
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_lidar_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_pub_;
  rclcpp::Subscription<rosflight_msgs::msg::SimState>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr lidar_timer_;

  // --- State ---
  rosflight_msgs::msg::SimState latest_state_;
  bool state_received_{false};
  bool sensor_frame_output_{false};  ///< lidar_output_frame == "sensor"

  // --- Obstacle storage ---
  std::vector<AABB>     boxes_;
  std::vector<Cylinder> cylinders_;

  // --- Collision accounting (truth vs obstacles; the sim has no physics) ---
  bool   collision_enabled_{true};
  double collision_radius_{0.49};   ///< vehicle hull; contact when centre is within this
  bool   in_collision_{false};      ///< edge-triggers the event counter and the log
  int    collision_events_{0};      ///< distinct entries into contact, not samples
  double deepest_penetration_{0.0}; ///< m, worst overlap seen this run
  double collision_seconds_{0.0};   ///< accumulated time in contact
  rclcpp::Time last_collision_check_;
  bool   have_last_check_{false};

  // --- Ground plane ---
  bool  ground_enabled_{true};
  double ground_z_{0.0};  // NED z-value of the ground plane (0 = takeoff level)
};

} // namespace rosflight_sim

#endif // MOCK_ENVIRONMENT_NODE_HPP
