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
  sensor_lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/livox/lidar", 10);

  const std::string output_frame = this->get_parameter("lidar_output_frame").as_string();
  if (output_frame != "ned" && output_frame != "sensor") {
    RCLCPP_ERROR(this->get_logger(), "lidar_output_frame '%s' is not 'ned' or 'sensor'; using 'ned'.",
                 output_frame.c_str());
  }
  sensor_frame_output_ = (output_frame == "sensor");
  RCLCPP_INFO(this->get_logger(), "LiDAR output: %s",
              sensor_frame_output_ ? "SENSOR frame on /livox/lidar (needs lidar_preprocessor)"
                                   : "world NED on mock_env/lidar (truth pose)");

  collision_pub_ = this->create_publisher<std_msgs::msg::Bool>("mock_env/collision", 10);

  collision_enabled_ = this->get_parameter("collision_check_enabled").as_bool();
  collision_radius_  = this->get_parameter("collision_radius").as_double();

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
  RCLCPP_INFO(this->get_logger(),
    "Collision reporting %s (hull %.2f m, checked against sim/truth_state, "
    "published on mock_env/collision).",
    collision_enabled_ ? "ON" : "OFF", collision_radius_);
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

  // Output frame of the scan.
  //   "ned"    -- hit points in world NED on mock_env/lidar, built from the TRUE pose. The
  //               historical behaviour and what the forest benchmark numbers were measured with.
  //               The planner's map is therefore always perfect, however wrong its estimate is.
  //   "sensor" -- the same hits in the LiDAR's own frame (x fwd, y left, z up, metres from the
  //               sensor) on /livox/lidar, stamped with the truth pose they were cast from --
  //               what livox_ros_driver2 publishes. Something downstream (lidar_preprocessor)
  //               must place them in the world using the ESTIMATED pose, as on the vehicle.
  // Assumes the LiDAR sits at the CG, upright and facing forward; the mounting offset is a
  // separate, later step.
  this->declare_parameter("lidar_output_frame", "ned");

  // Ground plane
  this->declare_parameter("ground_enabled", true);
  this->declare_parameter("ground_z_ned", 0.0);   // z=0 in NED = ground level

  // Obstacle world, as flat arrays because ROS 2 parameters cannot nest.
  //   cylinders: [cx, cy, cz, radius, half_height, ...]   (5 values each)
  //   boxes:     [minx, miny, minz, maxx, maxy, maxz, ...] (6 values each)
  // Leave BOTH empty to fall back to the built-in demo world, so existing launches are
  // unaffected. Populate them from a generated environment file to run a benchmark instead --
  // see tools/generate_forest.py.
  this->declare_parameter("cylinders", std::vector<double>{});
  this->declare_parameter("boxes", std::vector<double>{});

  // Collision reporting. The sim has no physics, so without this a vehicle that flies through a
  // trunk is indistinguishable from one that avoided it -- and a success rate defined as
  // "reached the goal WITHOUT collision" cannot be computed at all.
  this->declare_parameter("collision_check_enabled", true);
  // Vehicle hull radius. Should match the planner's drone_physical_radius (parsed from the
  // rotor geometry, 0.490 m for this airframe) so that "collided" means precisely "left the
  // volume the corridor certified", rather than some unrelated sphere.
  this->declare_parameter("collision_radius", 0.49);
}

// -------------------------------------------------------------------
// Build the obstacle world (hardcoded for now — easy to extend to YAML)
// -------------------------------------------------------------------
void MockEnvironmentNode::build_obstacle_world()
{
  ground_enabled_ = this->get_parameter("ground_enabled").as_bool();
  ground_z_       = this->get_parameter("ground_z_ned").as_double();

  // ------- Obstacles from parameters, if any were given -------
  const auto cyl = this->get_parameter("cylinders").as_double_array();
  const auto box = this->get_parameter("boxes").as_double_array();

  if (!cyl.empty() || !box.empty()) {
    if (cyl.size() % 5 != 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "'cylinders' has %zu values, not a multiple of 5 "
                   "([cx,cy,cz,radius,half_height] each). Ignoring it.", cyl.size());
    } else {
      for (std::size_t i = 0; i < cyl.size(); i += 5) {
        cylinders_.push_back({Eigen::Vector3d(cyl[i], cyl[i + 1], cyl[i + 2]),
                              cyl[i + 3], cyl[i + 4]});
      }
    }
    if (box.size() % 6 != 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "'boxes' has %zu values, not a multiple of 6 "
                   "([minx,miny,minz,maxx,maxy,maxz] each). Ignoring it.", box.size());
    } else {
      for (std::size_t i = 0; i < box.size(); i += 6) {
        boxes_.push_back({Eigen::Vector3d(box[i], box[i + 1], box[i + 2]),
                          Eigen::Vector3d(box[i + 3], box[i + 4], box[i + 5])});
      }
    }
    RCLCPP_INFO(this->get_logger(),
                "Obstacle world from parameters: %zu cylinder(s), %zu box(es), ground %s.",
                cylinders_.size(), boxes_.size(), ground_enabled_ ? "on" : "off");
    return;
  }

  RCLCPP_INFO(this->get_logger(),
              "No 'cylinders'/'boxes' parameters given; using the built-in demo world.");

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
  check_collision();
}

// -------------------------------------------------------------------
// Collision detection against the TRUE vehicle position
// -------------------------------------------------------------------
//
// Checked here, on truth, rather than anywhere in the autopilot: the planner and controller
// only ever see the ESTIMATED position, so a check on their side would report clearance from
// where the vehicle THINKS it is. A collision is a fact about the world, and truth is the only
// signal that carries it.
//
// Contact is centre-within-hull, i.e. the vehicle is modelled as a sphere of collision_radius.
// That is the same shape the planner inflates by, so "collided" here means exactly "left the
// volume the corridor certified" -- which is the property the safety argument claims.
//
// Events are edge-triggered. A single fly-through of one trunk lasts many state messages, and
// counting samples would report one long graze as hundreds of collisions and make the number
// meaningless for a success rate.
void MockEnvironmentNode::check_collision()
{
  if (!collision_enabled_ || !state_received_) { return; }

  const Eigen::Vector3d p(latest_state_.pose.position.x,
                          latest_state_.pose.position.y,
                          latest_state_.pose.position.z);

  double penetration = 0.0;   // how far INSIDE the hull-inflated obstacle the centre sits
  int hit_index = -1;

  for (std::size_t i = 0; i < cylinders_.size(); ++i) {
    const auto & c = cylinders_[i];
    // Outside the vertical extent of the trunk: no contact regardless of lateral distance.
    if (std::abs(p.z() - c.center.z()) > c.half_height + collision_radius_) { continue; }
    const double lateral = std::hypot(p.x() - c.center.x(), p.y() - c.center.y());
    const double overlap = (c.radius + collision_radius_) - lateral;
    if (overlap > penetration) { penetration = overlap; hit_index = static_cast<int>(i); }
  }

  for (const auto & b : boxes_) {
    // Distance from the centre to the box, per axis; zero inside.
    Eigen::Vector3d d = Eigen::Vector3d::Zero();
    for (int k = 0; k < 3; ++k) {
      d[k] = std::max({b.min_corner[k] - p[k], 0.0, p[k] - b.max_corner[k]});
    }
    const double overlap = collision_radius_ - d.norm();
    if (overlap > penetration) { penetration = overlap; hit_index = -2; }
  }

  const bool now_colliding = penetration > 0.0;

  // Time in contact, integrated on the state stream's own clock.
  const rclcpp::Time now = this->get_clock()->now();
  if (have_last_check_ && in_collision_) {
    const double dt = (now - last_collision_check_).seconds();
    if (dt > 0.0 && dt < 1.0) { collision_seconds_ += dt; }
  }
  last_collision_check_ = now;
  have_last_check_ = true;

  if (now_colliding) {
    deepest_penetration_ = std::max(deepest_penetration_, penetration);
    if (!in_collision_) {
      ++collision_events_;
      RCLCPP_ERROR(this->get_logger(),
                   "COLLISION %d: vehicle at (%.2f, %.2f, %.2f) is %.3f m inside %s "
                   "(hull %.2f m). Total time in contact %.2f s, deepest %.3f m.",
                   collision_events_, p.x(), p.y(), p.z(), penetration,
                   hit_index >= 0 ? "a cylinder" : "a box",
                   collision_radius_, collision_seconds_, deepest_penetration_);
    }
  } else if (in_collision_) {
    RCLCPP_WARN(this->get_logger(),
                "Clear of obstacle after collision %d. Time in contact so far %.2f s.",
                collision_events_, collision_seconds_);
  }
  in_collision_ = now_colliding;

  std_msgs::msg::Bool msg;
  msg.data = now_colliding;
  collision_pub_->publish(msg);
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

  if (sensor_frame_output_) {
    // Re-express each hit relative to the sensor: NED -> body FRD -> sensor FLU (180 deg roll).
    // Stamped with the pose the rays were cast from, so the preprocessor can look up the
    // estimate for the same instant rather than whatever is newest when the scan arrives.
    const Eigen::Matrix3d R_body_ned = q_body_to_ned.toRotationMatrix().transpose();
    const Eigen::Vector3d flu_from_frd(1.0, -1.0, -1.0);
    cloud_msg.header.stamp = latest_state_.header.stamp;
    cloud_msg.header.frame_id = "livox_frame";
    for (const auto & pt : hits) {
      const Eigen::Vector3d p_sensor = flu_from_frd.cwiseProduct(R_body_ned * (pt - origin));
      *iter_x = static_cast<float>(p_sensor.x());
      *iter_y = static_cast<float>(p_sensor.y());
      *iter_z = static_cast<float>(p_sensor.z());
      ++iter_x; ++iter_y; ++iter_z;
    }
    sensor_lidar_pub_->publish(cloud_msg);
    return;
  }

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

    // Neutral gray. Obstacles were reddish (0.8/0.3/0.2), which competed with the SFC wireframes
    // -- those are deliberately red, and two reds in one figure make it hard to tell the
    // corridor from the thing it is avoiding. Gray also reproduces legibly in print and in
    // greyscale. Opaque: RViz only shades solids, so at alpha 1.0 a trunk reads as an object
    // and an SFC wireframe cutting into one is obvious. The cost is that a trajectory passing
    // BEHIND a trunk is hidden from that viewpoint -- drop alpha to 0.7 to see through them.
    m.color.r = 0.5f;
    m.color.g = 0.5f;
    m.color.b = 0.5f;
    m.color.a = 1.0f;

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
