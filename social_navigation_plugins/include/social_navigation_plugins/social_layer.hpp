// Copyright 2019 Intelligent Robotics Lab
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

#ifndef SOCIAL_NAV_PLUGINS__SOCIAL_LAYER_HPP_
#define SOCIAL_NAV_PLUGINS__SOCIAL_LAYER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreorder"
#include "tf2_ros/message_filter.h"
#pragma GCC diagnostic pop
#include "message_filters/subscriber.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_costmap_2d/footprint.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "diagnostic_msgs/msg/key_value.hpp"

#include "social_navigation_plugins/geometry/geometry.hpp"

using KeyValue = diagnostic_msgs::msg::KeyValue;

namespace nav2_costmap_2d
{

class SocialLayer : public CostmapLayer
{
public:
  SocialLayer()
  {
    costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
  }

  struct Agent
  {
    std::string action;
    tf2::Transform tf;
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

  /**
   * @brief  A callback to handle tf message and know how many agents there are.
   * @param message The message returned from a message notifier
   */
  void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  void setActionCallback(const KeyValue::SharedPtr msg);

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
  
  std::vector<geometry_msgs::msg::Point> makeEscortFootprint(float r);
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
  rclcpp::Node::SharedPtr private_node_;
  std::vector<geometry_msgs::msg::Point> transformed_footprint_;
  std::map<std::string, Agent> agents_;
  std::vector<std::string> agent_ids_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<KeyValue>::SharedPtr set_action_sub_;
  std::string global_frame_;  ///< @brief The global frame for the costmap
  bool footprint_clearing_enabled_, rolling_window_, use_proxemics_, orientation_info_;
  std::string tf_prefix_;
  float intimate_z_radius_, social_z_radius_, gaussian_amplitude_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace nav2_costmap_2d

#endif  // SOCIAL_NAV_PLUGINS__SOCIAL_LAYER_HPP_
