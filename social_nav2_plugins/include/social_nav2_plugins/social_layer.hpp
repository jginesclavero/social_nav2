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

#ifndef social_nav2_PLUGINS__SOCIAL_LAYER_HPP_
#define social_nav2_PLUGINS__SOCIAL_LAYER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreorder"
#pragma GCC diagnostic pop
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "social_nav2_msgs/msg/set_human_action.hpp"

#include "social_nav2_plugins/geometry/geometry.hpp"

using SetHumanAction = social_nav2_msgs::msg::SetHumanAction;

namespace nav2_costmap_2d
{

class SocialLayer : public CostmapLayer
{
public:
  SocialLayer()
  : agents_()
  {
    costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
  }

  struct Agent
  {
    std::string action;
    tf2::Transform tf;
  };

  struct ActionZoneParams
  {
    float var_h;
    float var_r;
    float var_s;
    int n_activity_zones;
    float activity_zone_alpha;
    float activity_zone_phi;
  };

  virtual ~SocialLayer();
  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void activate();
  virtual void deactivate();
  virtual void reset();
  virtual bool isClearable() {return true;}

  /**
   * @brief  A callback to handle tf message and know how many agents there are.
   * @param message The message returned from a message notifier
   */
  void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  void setActionCallback(const SetHumanAction::SharedPtr msg);

protected:
  void doTouch(
    tf2::Transform agent, double * min_x, double * min_y,
    double * max_x, double * max_y);
  bool updateAgentMap(std::map<std::string, Agent> & agents);
  void updateFootprint(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y);
  void setProxemics(
    Agent & agent, float r, float amplitude);
  std::vector<geometry_msgs::msg::Point> makeEscortFootprint(float r, float alpha);
  void quarterFootprint(
    float r,
    float orientation,
    std::vector<geometry_msgs::msg::Point> & points);
  tf2::Vector3 transformPoint(
    const tf2::Vector3 & input_point, const tf2::Transform & transform);
  void transformProxemicFootprint(
    std::vector<geometry_msgs::msg::Point> input_points,
    tf2::Transform tf,
    std::vector<geometry_msgs::msg::Point> & transformed_proxemic,
    float alpha_mod = 0.0);
  void onParameterEventCallback(
    const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

  std::vector<geometry_msgs::msg::Point> transformed_footprint_;
  std::map<std::string, Agent> agents_;
  std::map<std::string, ActionZoneParams> action_z_params_map_;
  std::vector<std::string> action_names_;
  rclcpp::Subscription<SetHumanAction>::SharedPtr set_action_sub_;
  std::string global_frame_;  ///< @brief The global frame for the costmap
  bool footprint_clearing_enabled_, rolling_window_, orientation_info_, debug_only_;
  std::string tf_prefix_;
  float intimate_z_radius_, personal_z_radius_, gaussian_amplitude_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<nav2_costmap_2d::Costmap2D> social_costmap_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DPublisher> costmap_pub_{nullptr};};
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

}  // namespace nav2_costmap_2d

#endif  // social_nav2_PLUGINS__SOCIAL_LAYER_HPP_
