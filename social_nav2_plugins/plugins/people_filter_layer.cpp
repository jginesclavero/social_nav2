/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2020, Samsung R&D Institute Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 * 
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
 *********************************************************************/

// Copyright 2019 Intelligent Robotics Lab
// Author: jginesclavero https://github.com/jginesclavero/social_nav2


#include "social_nav2_plugins/people_filter_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

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
  auto node = node_.lock();
  // TODO(mjeronimo): these four are candidates for dynamic update
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("tf_prefix", rclcpp::ParameterValue("agent_"));
  declareParameter("filter_radius", rclcpp::ParameterValue(0.32));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "tf_prefix", tf_prefix_);
  node->get_parameter(name_ + "." + "filter_radius", filter_radius_);
  global_frame_ = layered_costmap_->getGlobalFrameID();
  rolling_window_ = layered_costmap_->isRolling();
  // default_value_ = NO_INFORMATION;
  RCLCPP_INFO(
    node->get_logger(),
    "Subscribed to TF Agent with prefix [%s] in global frame [%s]",
    tf_prefix_.c_str(), global_frame_.c_str());

  PeopleFilterLayer::matchSize();
  current_ = true;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(),
    node->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void
PeopleFilterLayer::getFrameNames()
{
  auto frames = tf_buffer_->getAllFrameNames();
  for (auto tf : frames) {    
    if (tf.find(tf_prefix_) != std::string::npos) {
      agent_ids_.push_back(tf);
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
  auto node = node_.lock();
  (void) robot_yaw;

  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  if (!enabled_) {return;}
  getFrameNames();
  clearArea(0, 0, getSizeInCellsX(), getSizeInCellsY(), true);
  useExtraBounds(min_x, min_y, max_x, max_y);
  bool current = true;

  // get the transform from the agents
  std::vector<tf2::Transform> agents;
  current = current && getAgentTFs(agents);

  // update the global current status
  current_ = current;
  if (agents.size() == 0) {
    RCLCPP_WARN(node->get_logger(), "Agents vector size is 0");
  }

  for (auto agent : agents) {
    agentFilter(agent, filter_radius_);
    doTouch(agent, min_x, min_y, max_x, max_y);
  }
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
  // master_array - is a direct pointer to the resulting master_grid.
  // master_grid - is a resulting costmap combined from all layers.
  // By using this pointer all layers will be overwritten!
  // To work with costmap layer and merge it with other costmap layers,
  // please use costmap_ pointer instead (this is pointer to current
  // costmap layer grid) and then call one of updates methods:
  // - updateWithAddition()
  // - updateWithMax()
  // - updateWithOverwrite()
  // - updateWithTrueOverwrite()
  // In this case using master_array pointer is equal to modifying local costmap_
  // pointer and then calling updateWithTrueOverwrite():
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
  // These variables are used to update the costmap only within this window
  // avoiding the updates of whole area.
  //
  // Fixing window coordinates with map size if necessary.
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
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
  auto node = node_.lock();
  geometry_msgs::msg::TransformStamped global2agent;
  for (auto id : agent_ids_) {
    try {
      // Check if the transform is available
      global2agent = tf_buffer_->lookupTransform(global_frame_, id, tf2::TimePointZero);
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(node->get_logger(), "%s", e.what());
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
    social_geometry::makeProxemicShapeFromAngle(r, 2 * M_PI),
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

// This is the macro allowing a nav2_gradient_costmap_plugin::GradientLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::PeopleFilterLayer, nav2_costmap_2d::Layer)