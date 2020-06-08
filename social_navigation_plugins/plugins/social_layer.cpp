// Copyright 2020 Intelligent Robotics Lab
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
// limitations under the License.

// Author: jginesclavero

#include "social_navigation_plugins/social_layer.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::SocialLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using KeyValue = diagnostic_msgs::msg::KeyValue;

namespace nav2_costmap_2d
{

SocialLayer::~SocialLayer() {}

void
SocialLayer::onInitialize()
{
  // TODO(mjeronimo): these four are candidates for dynamic update
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter(
    "footprint_clearing_enabled",
    rclcpp::ParameterValue(false));
  declareParameter("tf_prefix", rclcpp::ParameterValue("agent_"));
  declareParameter("intimate_z_radius", rclcpp::ParameterValue(0.32));
  declareParameter("personal_z_radius", rclcpp::ParameterValue(0.7));
  declareParameter("orientation_info", rclcpp::ParameterValue(false));
  declareParameter("use_proxemics", rclcpp::ParameterValue(true));

  node_->get_parameter(name_ + "." + "enabled", enabled_);
  node_->get_parameter(
    name_ + "." + "footprint_clearing_enabled",
    footprint_clearing_enabled_);
  node_->get_parameter(name_ + "." + "tf_prefix", tf_prefix_);
  node_->get_parameter(name_ + "." + "intimate_z_radius", intimate_z_radius_);
  node_->get_parameter(name_ + "." + "personal_z_radius", personal_z_radius_);
  node_->get_parameter(name_ + "." + "orientation_info", orientation_info_);
  node_->get_parameter(name_ + "." + "use_proxemics", use_proxemics_);

  global_frame_ = layered_costmap_->getGlobalFrameID();
  rolling_window_ = layered_costmap_->isRolling();
  default_value_ = NO_INFORMATION;
  gaussian_amplitude_ = 400.0; /* Amplitude value to get a LETHAL_OBSTACLE intimate zone */
  RCLCPP_INFO(node_->get_logger(),
    "Subscribed to TF Agent with prefix [%s] in global frame [%s]",
    tf_prefix_.c_str(), global_frame_.c_str());
  private_node_ = rclcpp::Node::make_shared("social_layer_sub");

  SocialLayer::matchSize();
  current_ = true;
  tf_received_= false;

  tf_sub_ = private_node_->create_subscription<tf2_msgs::msg::TFMessage>(
    "tf", rclcpp::SensorDataQoS(),
    std::bind(&SocialLayer::tfCallback, this, std::placeholders::_1));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node_->get_node_base_interface(),
    node_->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  set_action_sub_ = private_node_->create_subscription<KeyValue>(
    "social_navigation/set_agent_action",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&SocialLayer::setActionCallback, this, std::placeholders::_1));
}

void
SocialLayer::tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  for (auto tf : msg->transforms) {
    if (tf.child_frame_id.find(tf_prefix_) != std::string::npos &&
      agents_.find(tf.child_frame_id) == agents_.end())
    {
      Agent a;
      agents_.insert(std::pair<std::string, Agent>(tf.child_frame_id, a));
      tf_received_ = true;
    }
  }
}

void SocialLayer::setActionCallback(const KeyValue::SharedPtr msg)
{
  agents_[msg->key].action = msg->value;
}

void
SocialLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_ || !tf_received_) {return;}

  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  clearArea(0, 0, getSizeInCellsX(), getSizeInCellsY());

  useExtraBounds(min_x, min_y, max_x, max_y);
  bool current = true;

  // get the transform from the agents
  current = current && updateAgentMap(agents_);

  // update the global current status
  current_ = current;

  for (auto agent : agents_) {
    if (use_proxemics_) {
      setProxemics(agent.second, personal_z_radius_, gaussian_amplitude_);
    }
    doTouch(agent.second.tf, min_x, min_y, max_x, max_y);
  }

  if (footprint_clearing_enabled_) {
    setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
  }
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void
SocialLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!footprint_clearing_enabled_) {return;}
  transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

  for (unsigned int i = 0; i < transformed_footprint_.size(); i++) {
    touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}

void
SocialLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {return;}
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  rclcpp::spin_some(private_node_);
}

void
SocialLayer::doTouch(
  tf2::Transform agent,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::vector<geometry_msgs::msg::Point> footprint;
  transformFootprint(
    agent.getOrigin().x(),
    agent.getOrigin().y(),
    agent.getRotation().getAngle(),
    makeFootprintFromRadius(intimate_z_radius_),
    footprint);
  for (unsigned int i = 0; i < footprint.size(); i++) {
    touch(footprint[i].x, footprint[i].y, min_x, min_y, max_x, max_y);
  }
}

bool
SocialLayer::updateAgentMap(std::map<std::string, Agent> & agents)
{
  geometry_msgs::msg::TransformStamped global2agent;
  for (auto agent : agents) {
    try {
      // Check if the transform is available
      global2agent = tf_buffer_->lookupTransform(global_frame_, agent.first, tf2::TimePointZero);
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(node_->get_logger(), "%s", e.what());
      return false;
    }
    tf2::Transform global2agent_tf2;
    tf2::impl::Converter<true, false>::convert(global2agent.transform, global2agent_tf2);
    agent.second.tf = global2agent_tf2;
    agents[agent.first] = agent.second;
    //RCLCPP_INFO(node_->get_logger(), "P [%f %f]", agent.second.tf.getOrigin().getX(), agent.second.tf.getOrigin().getY());
  }
  return true;
}

void
SocialLayer::setProxemics(
  Agent & agent, float r, float amplitude)
{
  std::vector<geometry_msgs::msg::Point> agent_footprint, intimate_footprint;
  std::vector<MapLocation> polygon_cells;
  double var_h, var_s, var_r;
  var_h = 0.8;
  var_s = 0.4;
  var_r = 0.2;
  tf2::Matrix3x3 m(agent.tf.getRotation());
  double roll, pitch, yaw;
  if (orientation_info_) {
    m.getRPY(roll, pitch, yaw);
  } else {
    yaw = 0.0;
  }

  float proxemic_mod_angle = 2 * M_PI - M_PI / 4;
  if (orientation_info_) {
    if (agent.action == "guiding") {
      // Proxemics for Guiding = HRI?
      transformProxemicFootprint(
        social_geometry::makeProxemicShapeFromAngle(
          r, proxemic_mod_angle, M_PI / 6),
        agent.tf,
        agent_footprint);
    } else if (agent.action == "following") {
      var_h = 0.5;
      var_s = 0.5;
      var_r = 0.35;
      // Proxemics for Follow
      transformProxemicFootprint(
        social_geometry::makeProxemicShapeFromAngle(
          r, proxemic_mod_angle, M_PI / 6 + M_PI),
        agent.tf,
        agent_footprint);
    } else if (agent.action == "escorting") {
      var_h = 0.5;
      var_s = 0.5;
      var_r = 0.5;
      // Proxemics for Escort
      transformProxemicFootprint(
        makeEscortFootprint(r),
        agent.tf,
        agent_footprint);
    } else {
      var_h = 0.35;
      var_s = 0.5;
      var_r = 0.5;
      // Proxemics for HRI
      transformProxemicFootprint(
        social_geometry::makeProxemicShapeFromAngle(
          r, proxemic_mod_angle, M_PI / 6),
        agent.tf,
        agent_footprint);
    }
  } else {
    // Classic proxemic approach
    var_h = 0.5;
    var_s = 0.5;
    var_r = 0.5;
    transformProxemicFootprint(
      social_geometry::makeProxemicShapeFromAngle(r, 2 * M_PI),
      agent.tf,
      agent_footprint);
  }

  social_geometry::getPolygon(
    layered_costmap_->getCostmap(),
    agent_footprint,
    polygon_cells);

  // We add the intimate zone footprint to the proxemic shape polygon.
  transformProxemicFootprint(
    social_geometry::makeProxemicShapeFromAngle(intimate_z_radius_, 2 * M_PI),
    agent.tf,
    intimate_footprint);
  social_geometry::getPolygon(
    layered_costmap_->getCostmap(),
    intimate_footprint,
    polygon_cells);

  for (unsigned int i = 0; i < polygon_cells.size(); i++) {
    double ax, ay;
    mapToWorld(polygon_cells[i].x, polygon_cells[i].y, ax, ay);
    double a = social_geometry::asymmetricGaussian(
      ax,
      ay,
      agent.tf.getOrigin().x(),
      agent.tf.getOrigin().y(),
      amplitude,
      yaw,
      var_h,
      var_s,
      var_r);
    if (a > 254.0) {a = 254.0;}
    if (a <= 20.0) {continue;}
    unsigned int index = getIndex(polygon_cells[i].x, polygon_cells[i].y);
    unsigned char cvalue = (unsigned char) a;
    unsigned char old_value = costmap_[index];
    if (old_value < 255) {
      costmap_[index] = std::max(old_value, cvalue);
    } else {
      costmap_[index] = cvalue;
    }
    //RCLCPP_INFO(node_->get_logger(), "index %u value %lu", index, cvalue);
  }
}

tf2::Vector3
SocialLayer::transformPoint(
  const tf2::Vector3 & input_point, const tf2::Transform & transform)
{
  return transform * input_point;
}

void
SocialLayer::transformProxemicFootprint(
  std::vector<geometry_msgs::msg::Point> input_points,
  tf2::Transform tf,
  std::vector<geometry_msgs::msg::Point> & transformed_proxemic,
  float alpha_mod)
{
  tf2::Transform tf_ = tf;
  tf2::Quaternion q_orig, q_rot, q_new;
  q_rot.setRPY(alpha_mod, 0, 0);
  q_new = q_rot * tf_.getRotation();  // Calculate the new orientation
  q_new.normalize();
  tf_.setRotation(q_new);

  for (auto p : input_points) {
    tf2::Vector3 p_obj(p.x, p.y, 0.0);
    auto transform_p = transformPoint(p_obj, tf_);
    geometry_msgs::msg::Point out_p;
    out_p.x = transform_p.x();
    out_p.y = transform_p.y();
    transformed_proxemic.push_back(out_p);
  }
}

std::vector<geometry_msgs::msg::Point>
SocialLayer::makeEscortFootprint(float r)
{
  std::vector<geometry_msgs::msg::Point> points;
  geometry_msgs::msg::Point pt;
  pt.x = 0.0;
  pt.y = 0.0;
  points.push_back(pt);
  float orientation = -M_PI / 4;
  quarterFootprint(r, orientation, points);
  orientation = orientation + M_PI;
  quarterFootprint(r, orientation, points);
  return points;
}

void
SocialLayer::quarterFootprint(
  float r, float orientation, std::vector<geometry_msgs::msg::Point> & points)
{
  // Loop over 32 angles around a circle making a point each time
  int N = 32;
  float alpha = M_PI / 2;
  int it = (int) round((N * alpha) / (2 * M_PI));
  geometry_msgs::msg::Point pt;
  for (int i = 0; i < it; ++i) {
    double angle = i * 2 * M_PI / N + orientation;
    pt.x = cos(angle) * r;
    pt.y = sin(angle) * r;
    points.push_back(pt);
  }
  pt.x = points[0].x;
  pt.y = points[0].y;
  points.push_back(pt);
}

void
SocialLayer::activate() {}

void
SocialLayer::deactivate() {}

void
SocialLayer::reset()
{
  resetMaps();
  current_ = true;
}

}  // namespace nav2_costmap_2d
