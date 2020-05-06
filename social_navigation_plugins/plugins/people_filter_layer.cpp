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

#include "social_navigation_plugins/people_filter_layer.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::PeopleFilterLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

namespace nav2_costmap_2d
{

PeopleFilterLayer::~PeopleFilterLayer() {}

void
PeopleFilterLayer::onInitialize()
{
  // TODO(mjeronimo): these four are candidates for dynamic update
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("tf_prefix", rclcpp::ParameterValue("agent_"));
  declareParameter("filter_radius", rclcpp::ParameterValue(0.32));

  node_->get_parameter(name_ + "." + "enabled", enabled_);
  node_->get_parameter(name_ + "." + "tf_prefix", tf_prefix_);
  node_->get_parameter(name_ + "." + "filter_radius", filter_radius_);
  
  global_frame_ = layered_costmap_->getGlobalFrameID();
  rolling_window_ = layered_costmap_->isRolling();
  default_value_ = NO_INFORMATION;
  
  RCLCPP_INFO(node_->get_logger(),
    "Subscribed to TF Agent with prefix [%s] in global frame [%s]",
    tf_prefix_.c_str(), global_frame_.c_str());
  private_node_ = rclcpp::Node::make_shared("people_filter_layer_sub");

  PeopleFilterLayer::matchSize();
  current_ = true;

  tf_sub_ = private_node_->create_subscription<tf2_msgs::msg::TFMessage>(
    "tf", rclcpp::SensorDataQoS(),
    std::bind(&PeopleFilterLayer::tfCallback, this, std::placeholders::_1));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node_->get_node_base_interface(),
    node_->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void
PeopleFilterLayer::tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  for (auto tf : msg->transforms) {
    if (tf.child_frame_id.find(tf_prefix_) != std::string::npos) {
      agent_ids_.push_back(tf.child_frame_id);
    }
  }
  sort(agent_ids_.begin(), agent_ids_.end());
  agent_ids_.erase(unique(agent_ids_.begin(), agent_ids_.end()), agent_ids_.end());
}

void
PeopleFilterLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  if (!enabled_) {return;}
  clearArea(0, 0, getSizeInCellsX(), getSizeInCellsY());

  useExtraBounds(min_x, min_y, max_x, max_y);
  bool current = true;

  // get the transform from the agents
  std::vector<tf2::Transform> agents;
  current = current && getAgentTFs(agents);

  // update the global current status
  current_ = current;

  for (auto agent : agents) {
    agentFilter(agent, filter_radius_);
    doTouch(agent, min_x, min_y, max_x, max_y);
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void
PeopleFilterLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);
  for (unsigned int i = 0; i < transformed_footprint_.size(); i++) {
    touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}

void
PeopleFilterLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {return;}
  updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
  rclcpp::spin_some(private_node_);
}

void
PeopleFilterLayer::doTouch(
  tf2::Transform agent,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::vector<geometry_msgs::msg::Point> footprint;
  transformFootprint(
    agent.getOrigin().x(),
    agent.getOrigin().y(),
    agent.getRotation().getAngle(),
    makeFootprintFromRadius(filter_radius_),
    footprint);

  for (unsigned int i = 0; i < footprint.size(); i++) {
    touch(footprint[i].x, footprint[i].y, min_x, min_y, max_x, max_y);
  }
}

bool
PeopleFilterLayer::getAgentTFs(std::vector<tf2::Transform> & agents) const
{
  geometry_msgs::msg::TransformStamped global2agent;

  for (auto id : agent_ids_) {
    try {
      // Check if the transform is available
      global2agent = tf_buffer_->lookupTransform(global_frame_, id, tf2::TimePointZero);
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(node_->get_logger(), "%s", e.what());
      return false;
    }
    tf2::Transform global2agent_tf2;
    tf2::impl::Converter<true, false>::convert(global2agent.transform, global2agent_tf2);
    agents.push_back(global2agent_tf2);
  }
  return true;
}

void
PeopleFilterLayer::agentFilter(tf2::Transform agent, float r)
{
  std::vector<geometry_msgs::msg::Point> agent_footprint;
  transformFootprint(
    agent.getOrigin().x(),
    agent.getOrigin().y(),
    agent.getRotation().getAngle(),
    makeFootprintFromRadius(r),
    agent_footprint);
  std::vector<MapLocation> polygon_cells;
  social_geometry::getPolygon(
    layered_costmap_->getCostmap(),
    agent_footprint,
    polygon_cells);
  for (unsigned int i = 0; i < polygon_cells.size(); ++i) {
    unsigned int index = getIndex(polygon_cells[i].x, polygon_cells[i].y);
    costmap_[index] = nav2_costmap_2d::FREE_SPACE;
  }
}

void
PeopleFilterLayer::activate() {}

void
PeopleFilterLayer::deactivate() {}

void
PeopleFilterLayer::reset()
{
  resetMaps();
  current_ = true;
}

}  // namespace nav2_costmap_2d
