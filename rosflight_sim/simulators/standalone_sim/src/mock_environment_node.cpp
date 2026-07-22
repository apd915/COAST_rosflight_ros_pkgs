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

#include "mock_environment_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace rosflight_sim
{

// ===================================================================
// Ray–primitive intersection helpers
// ===================================================================

double AABB::intersect(const Eigen::Vector3d & origin,
                       const Eigen::Vector3d & dir) const
{
  // Slab method — works for axis-aligned boxes.
  double tmin = -std::numeric_limits<double>::infinity();
  double tmax =  std::numeric_limits<double>::infinity();

  for (int i = 0; i < 3; ++i) {
    if (std::abs(dir[i]) < 1e-12) {
      // Ray is parallel to this slab
      if (origin[i] < min_corner[i] || origin[i] > max_corner[i]) {
        return -1.0;
      }
    } else {
      double inv_d = 1.0 / dir[i];
      double t1 = (min_corner[i] - origin[i]) * inv_d;
      double t2 = (max_corner[i] - origin[i]) * inv_d;
      if (t1 > t2) std::swap(t1, t2);
      tmin = std::max(tmin, t1);
      tmax = std::min(tmax, t2);
      if (tmin > tmax) return -1.0;
    }
  }
  return (tmin >= 0.0) ? tmin : ((tmax >= 0.0) ? tmax : -1.0);
}

double Cylinder::intersect(const Eigen::Vector3d & origin,
                           const Eigen::Vector3d & dir) const
{
  // Infinite cylinder aligned with z-NED, then cap check.
  double dx = origin.x() - center.x();
  double dy = origin.y() - center.y();

  double a = dir.x() * dir.x() + dir.y() * dir.y();
  double b = 2.0 * (dx * dir.x() + dy * dir.y());
  double c = dx * dx + dy * dy - radius * radius;

  double disc = b * b - 4.0 * a * c;
  if (disc < 0.0) return -1.0;

  double sqrt_disc = std::sqrt(disc);
  double t0 = (-b - sqrt_disc) / (2.0 * a);
  double t1 = (-b + sqrt_disc) / (2.0 * a);

  // We want the nearest positive t that is within the height caps.
  double z_min = center.z() - half_height;
  double z_max = center.z() + half_height;

  auto in_caps = [&](double t) -> bool {
    double z = origin.z() + t * dir.z();
    return z >= z_min && z <= z_max;
  };

  if (t0 >= 0.0 && in_caps(t0)) return t0;
  if (t1 >= 0.0 && in_caps(t1)) return t1;

  // Check cap planes (top and bottom discs).
  if (std::abs(dir.z()) > 1e-12) {
    for (double cap_z : {z_min, z_max}) {
      double t_cap = (cap_z - origin.z()) / dir.z();
      if (t_cap >= 0.0) {
        double hx = origin.x() + t_cap * dir.x() - center.x();
        double hy = origin.y() + t_cap * dir.y() - center.y();
        if (hx * hx + hy * hy <= radius * radius) {
          return t_cap;
        }
      }
    }
  }

  return -1.0;
}

// ===================================================================
// MockEnvironmentNode implementation
// ===================================================================

MockEnvironmentNode::MockEnvironmentNode()
  : Node("mock_environment")
{
  declare_parameters();
  build_obstacle_world();

  // --- Publishers ---
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "mock_env/obstacles", rclcpp::QoS(1).transient_local());

  lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "mock_env/lidar", 10);

  // --- Subscriber ---
  state_sub_ = this->create_subscription<rosflight_msgs::msg::SimState>(
    "sim/truth_state", 10,
    std::bind(&MockEnvironmentNode::state_callback, this, std::placeholders::_1));

  // --- Timer for LiDAR scan (10 Hz default) ---
  double lidar_rate_hz = this->get_parameter("lidar_rate_hz").as_double();
  auto period = std::chrono::duration<double>(1.0 / lidar_rate_hz);
  lidar_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&MockEnvironmentNode::lidar_timer_callback, this));

  // Publish obstacle markers once (latched via transient_local QoS)
  publish_obstacle_markers();

  RCLCPP_INFO(this->get_logger(),
    "MockEnvironmentNode started — %zu boxes, %zu cylinders, ground=%s",
    boxes_.size(), cylinders_.size(),
    ground_enabled_ ? "ON" : "OFF");
}

// -------------------------------------------------------------------
// Parameters
// -------------------------------------------------------------------
void MockEnvironmentNode::declare_parameters()
{
  // LiDAR configuration (Livox Mid-360 specs)
  this->declare_parameter("lidar_rate_hz", 10.0);
  this->declare_parameter("lidar_min_range", 0.1);   // physical blind zone
  this->declare_parameter("lidar_max_range", 70.0);   // 80% reflectivity range

  // Horizontal scan: full 360 degrees
  this->declare_parameter("lidar_h_fov_deg", 360.0);
  this->declare_parameter("lidar_h_res_deg", 3.0);    // default coarse for sim performance

  // Vertical scan: asymmetric FOV (Livox Mid-360: -7° to +52°)
  this->declare_parameter("lidar_v_fov_min_deg", -7.0);
  this->declare_parameter("lidar_v_fov_max_deg", 52.0);
  this->declare_parameter("lidar_v_res_deg", 3.0);    // default coarse for sim performance

  // Ground plane
  this->declare_parameter("ground_enabled", true);
  this->declare_parameter("ground_z_ned", 0.0);   // z=0 in NED = ground level
}

// -------------------------------------------------------------------
// Build the obstacle world (hardcoded for now — easy to extend to YAML)
// -------------------------------------------------------------------
void MockEnvironmentNode::build_obstacle_world()
{
  ground_enabled_ = this->get_parameter("ground_enabled").as_bool();
  ground_z_       = this->get_parameter("ground_z_ned").as_double();

  // ------- Example obstacles (all in NED metres) -------
  // Note: in NED, negative z is UP.  So a pillar from ground (z=0)
  // to 8 m high would span z = 0  to  z = -8.

  // Pillar 1 — cylinder at (5, 3) NED, radius 0.5 m, 8 m tall
  cylinders_.push_back({
    Eigen::Vector3d(5.0, 3.0, -4.0),   // center (halfway up)
    0.5,                                 // radius
    4.0                                  // half_height
  });

  // Pillar 2 — cylinder at (8, -2) NED, radius 0.75 m, 6 m tall
  cylinders_.push_back({
    Eigen::Vector3d(8.0, -2.0, -3.0),
    0.75,
    3.0
  });

  // Wall segment — thin box
  boxes_.push_back({
    Eigen::Vector3d(12.0, -3.0, -6.0),  // min corner
    Eigen::Vector3d(12.5,  3.0,  0.0)   // max corner (0.5 m thick wall)
  });

  // Floating box obstacle
  boxes_.push_back({
    Eigen::Vector3d(6.0, -1.0, -5.0),
    Eigen::Vector3d(7.0,  1.0, -3.0)
  });

  // --- NEW OBSTACLES FOR 360 TESTING ---
  
  // Pillar behind the drone (Negative X)
  cylinders_.push_back({
    Eigen::Vector3d(-7.0, 1.0, -3.0),
    1.0,
    3.0
  });

  // Pillar to the left (Negative Y)
  cylinders_.push_back({
    Eigen::Vector3d(2.0, -8.0, -5.0),
    0.5,
    5.0
  });

  // Pillar to the right (Positive Y)
  cylinders_.push_back({
    Eigen::Vector3d(0.0, 9.0, -4.0),
    1.5,
    4.0
  });

  // Wall behind and to the right
  boxes_.push_back({
    Eigen::Vector3d(-5.0, 4.0, -4.0),
    Eigen::Vector3d(-4.5, 10.0, 0.0)
  });

  // Floating box to the left
  boxes_.push_back({
    Eigen::Vector3d(-2.0, -6.0, -8.0),
    Eigen::Vector3d(-1.0, -4.0, -6.0)
  });
}

// -------------------------------------------------------------------
// State callback
// -------------------------------------------------------------------
void MockEnvironmentNode::state_callback(
  const rosflight_msgs::msg::SimState & msg)
{
  latest_state_ = msg;
  state_received_ = true;
}

// -------------------------------------------------------------------
// Timer callback — triggers a full LiDAR sweep
// -------------------------------------------------------------------
void MockEnvironmentNode::lidar_timer_callback()
{
  if (!state_received_) return;
  publish_lidar_scan();
}

// -------------------------------------------------------------------
// Cast a single ray against every obstacle; return nearest hit distance.
// -------------------------------------------------------------------
double MockEnvironmentNode::cast_ray(
  const Eigen::Vector3d & origin,
  const Eigen::Vector3d & direction,
  double max_range) const
{
  double best_t = max_range;

  for (const auto & box : boxes_) {
    double t = box.intersect(origin, direction);
    if (t >= 0.0 && t < best_t) best_t = t;
  }

  for (const auto & cyl : cylinders_) {
    double t = cyl.intersect(origin, direction);
    if (t >= 0.0 && t < best_t) best_t = t;
  }

  // Ground plane at z = ground_z_ (NED, positive-z is down)
  if (ground_enabled_ && std::abs(direction.z()) > 1e-12) {
    double t_ground = (ground_z_ - origin.z()) / direction.z();
    if (t_ground >= 0.0 && t_ground < best_t) best_t = t_ground;
  }

  return best_t;
}

// -------------------------------------------------------------------
// Build and publish the PointCloud2
// -------------------------------------------------------------------
void MockEnvironmentNode::publish_lidar_scan()
{
  double min_range       = this->get_parameter("lidar_min_range").as_double();
  double max_range       = this->get_parameter("lidar_max_range").as_double();
  double h_fov_deg       = this->get_parameter("lidar_h_fov_deg").as_double();
  double h_res_deg       = this->get_parameter("lidar_h_res_deg").as_double();
  double v_fov_min_deg   = this->get_parameter("lidar_v_fov_min_deg").as_double();
  double v_fov_max_deg   = this->get_parameter("lidar_v_fov_max_deg").as_double();
  double v_res_deg       = this->get_parameter("lidar_v_res_deg").as_double();

  // Drone position in NED
  Eigen::Vector3d origin(
    latest_state_.pose.position.x,
    latest_state_.pose.position.y,
    latest_state_.pose.position.z);

  // Drone orientation (body-to-inertial quaternion)
  Eigen::Quaterniond q_body_to_ned(
    latest_state_.pose.orientation.w,
    latest_state_.pose.orientation.x,
    latest_state_.pose.orientation.y,
    latest_state_.pose.orientation.z);

  // Pre-compute ray counts (asymmetric vertical FOV)
  int n_h = static_cast<int>(std::ceil(h_fov_deg / h_res_deg));
  double v_start = v_fov_min_deg;
  double v_end   = v_fov_max_deg;
  int n_v = static_cast<int>(std::ceil((v_end - v_start) / v_res_deg));
  double h_start = -h_fov_deg / 2.0;

  // Collect hit points (in NED frame)
  std::vector<Eigen::Vector3d> hits;
  hits.reserve(n_h * n_v);

  for (int iv = 0; iv < n_v; ++iv) {
    double elev_deg = v_start + iv * v_res_deg;
    double elev_rad = elev_deg * M_PI / 180.0;

    for (int ih = 0; ih < n_h; ++ih) {
      double az_deg = h_start + ih * h_res_deg;
      double az_rad = az_deg * M_PI / 180.0;

      // Direction in body frame (x-forward, y-right, z-down in NED body).
      // NED convention: positive elevation = looking UP = negative z.
      Eigen::Vector3d dir_body(
        std::cos(elev_rad) * std::cos(az_rad),
        std::cos(elev_rad) * std::sin(az_rad),
        -std::sin(elev_rad));

      // Rotate to NED inertial frame
      Eigen::Vector3d dir_ned = q_body_to_ned * dir_body;
      dir_ned.normalize();

      double t = cast_ray(origin, dir_ned, max_range);
      if (t >= min_range && t < max_range - 1e-3) {
        hits.push_back(origin + t * dir_ned);
      }
    }
  }

  // --- Pack into PointCloud2 (XYZ float32, in NED frame) ---
  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header.stamp = this->get_clock()->now();
  cloud_msg.header.frame_id = "NED";
  cloud_msg.height = 1;
  cloud_msg.width  = static_cast<uint32_t>(hits.size());
  cloud_msg.is_dense = true;
  cloud_msg.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(hits.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

  for (const auto & pt : hits) {
    *iter_x = static_cast<float>(pt.x());
    *iter_y = static_cast<float>(pt.y());
    *iter_z = static_cast<float>(pt.z());
    ++iter_x; ++iter_y; ++iter_z;
  }

  lidar_pub_->publish(cloud_msg);
}

// -------------------------------------------------------------------
// Publish obstacle markers for RViz (transient_local → latched)
// -------------------------------------------------------------------
void MockEnvironmentNode::publish_obstacle_markers()
{
  visualization_msgs::msg::MarkerArray marker_array;
  int id = 0;

  // --- Boxes ---
  for (const auto & box : boxes_) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "NED";
    m.header.stamp = this->get_clock()->now();
    m.ns = "mock_env_boxes";
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;

    Eigen::Vector3d center = (box.min_corner + box.max_corner) / 2.0;
    Eigen::Vector3d size   = box.max_corner - box.min_corner;

    m.pose.position.x = center.x();
    m.pose.position.y = center.y();
    m.pose.position.z = center.z();
    m.pose.orientation.w = 1.0;

    m.scale.x = size.x();
    m.scale.y = size.y();
    m.scale.z = size.z();

    m.color.r = 0.6f;
    m.color.g = 0.6f;
    m.color.b = 0.6f;
    m.color.a = 0.7f;

    marker_array.markers.push_back(m);
  }

  // --- Cylinders ---
  for (const auto & cyl : cylinders_) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "NED";
    m.header.stamp = this->get_clock()->now();
    m.ns = "mock_env_cylinders";
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = cyl.center.x();
    m.pose.position.y = cyl.center.y();
    m.pose.position.z = cyl.center.z();
    m.pose.orientation.w = 1.0;

    m.scale.x = 2.0 * cyl.radius;   // diameter
    m.scale.y = 2.0 * cyl.radius;
    m.scale.z = 2.0 * cyl.half_height;

    m.color.r = 0.8f;
    m.color.g = 0.3f;
    m.color.b = 0.2f;
    m.color.a = 0.7f;

    marker_array.markers.push_back(m);
  }

  // --- Ground plane (thin flat box) ---
  if (ground_enabled_) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "NED";
    m.header.stamp = this->get_clock()->now();
    m.ns = "mock_env_ground";
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = ground_z_ + 0.005;  // just below z=0
    m.pose.orientation.w = 1.0;

    m.scale.x = 60.0;   // 60 × 60 m ground patch
    m.scale.y = 60.0;
    m.scale.z = 0.01;   // 1 cm thick

    m.color.r = 0.3f;
    m.color.g = 0.5f;
    m.color.b = 0.3f;
    m.color.a = 0.3f;

    marker_array.markers.push_back(m);
  }

  marker_pub_->publish(marker_array);
}

} // namespace rosflight_sim

// -------------------------------------------------------------------
// main
// -------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rosflight_sim::MockEnvironmentNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
