/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 * Author: Jonatan Gines
 *
 *********************************************************************/
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

namespace nav2_costmap_2d
{

PeopleFilterLayer::~PeopleFilterLayer(){}

void 
PeopleFilterLayer::onInitialize()
{
  // TODO(mjeronimo): these four are candidates for dynamic update
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter(
    "footprint_clearing_enabled",
    rclcpp::ParameterValue(true));
  declareParameter("tf_prefix", rclcpp::ParameterValue("agent_"));

  node_->get_parameter(name_ + "." + "enabled", enabled_);
  node_->get_parameter(name_ + "." + "footprint_clearing_enabled", footprint_clearing_enabled_);
  node_->get_parameter(name_ + "." + "tf_prefix", tf_prefix_);
  
  global_frame_ = layered_costmap_->getGlobalFrameID();
  rolling_window_ = layered_costmap_->isRolling();

  RCLCPP_INFO(node_->get_logger(), "Subscribed to TF Agent with prefix [%s] in global frame [%s]", tf_prefix_.c_str(), global_frame_.c_str());
  
  private_node_ = rclcpp::Node::make_shared("people_filter_layer_sub");
  default_value_ = NO_INFORMATION;

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
  for (auto tf : msg->transforms)
  { 
    if (tf.child_frame_id.find(tf_prefix_) != std::string::npos)
      agent_ids_.push_back(tf.child_frame_id);
  }
  sort(agent_ids_.begin(), agent_ids_.end());
  agent_ids_.erase(unique(agent_ids_.begin(), agent_ids_.end()), agent_ids_.end());
}


void
PeopleFilterLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  if (!enabled_) {
    return;
  }
  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<tf2::Transform> agents;

  // get the transform of the agents
  current = current && getAgentTFs(agents);

  // update the global current status
  current_ = current;

  for (auto agent : agents)
  {
    std::vector<geometry_msgs::msg::Point> agent_footprint_;
    
    transformFootprint(
      agent.getOrigin().x(),
      agent.getOrigin().y(),
      agent.getRotation().getAngle(),
      makeFootprintFromRadius(0.32),
      agent_footprint_);
    setConvexPolygonCost(agent_footprint_, nav2_costmap_2d::FREE_SPACE);
    
    for (unsigned int i = 0; i < agent_footprint_.size(); i++) {
      touch(agent_footprint_[i].x, agent_footprint_[i].y, min_x, min_y, max_x, max_y);
    }
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  rclcpp::spin_some(private_node_);
}

void
PeopleFilterLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x,
  double * max_y)
{
  if (!footprint_clearing_enabled_) {return;}
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
  if (!enabled_) {
    return;
  }

  if (footprint_clearing_enabled_) {
    setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
  }

  updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
}

bool
PeopleFilterLayer::getAgentTFs(std::vector<tf2::Transform> & agents) const
{
  geometry_msgs::msg::TransformStamped global2agent;

  for (auto id : agent_ids_)
  {
    try 
    {
      // Check if the transform is available
      global2agent = tf_buffer_->lookupTransform(global_frame_, id, tf2::TimePointZero);
    } 
    catch (tf2::TransformException &e) 
    {
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
PeopleFilterLayer::activate()
{
  //// if we're stopped we need to re-subscribe to topics
  //for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
  //  if (observation_subscribers_[i] != NULL) {
  //    observation_subscribers_[i]->subscribe();
  //  }
  //}
  //resetBuffersLastUpdated();
}

void
PeopleFilterLayer::deactivate()
{
  //for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
  //  if (observation_subscribers_[i] != NULL) {
  //    observation_subscribers_[i]->unsubscribe();
  //  }
  //}
}

void
PeopleFilterLayer::reset()
{
  resetMaps();
  //resetBuffersLastUpdated();
  current_ = true;
}

//void
//ObstacleLayer::resetBuffersLastUpdated()
//{
//  for (unsigned int i = 0; i < observation_buffers_.size(); ++i) {
//    if (observation_buffers_[i]) {
//      observation_buffers_[i]->resetLastUpdated();
//    }
//  }
//}

}  // namespace nav2_costmap_2d
