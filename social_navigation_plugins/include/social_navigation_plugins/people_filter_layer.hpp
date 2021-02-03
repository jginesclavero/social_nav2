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

#ifndef SOCIAL_NAVIGATION_PLUGINS__PEOPLE_FILTER_LAYER_HPP_
#define SOCIAL_NAVIGATION_PLUGINS__PEOPLE_FILTER_LAYER_HPP_

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

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "social_navigation_plugins/geometry/geometry.hpp"

namespace nav2_costmap_2d
{

class PeopleFilterLayer : public CostmapLayer
{
public:
  PeopleFilterLayer()
  {
    costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
  }

  virtual ~PeopleFilterLayer();
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
   * @brief  A method to get the framenames of the system know how many agents there are.
   */
  void getFrameNames();
  
protected:
  void doTouch(
    tf2::Transform agent, double * min_x, double * min_y,
    double * max_x, double * max_y);
  bool getAgentTFs(std::vector<tf2::Transform> & agents) const;
  void updateFootprint(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y);
  void agentFilter(tf2::Transform agent, float r);

  std::vector<geometry_msgs::msg::Point> transformed_footprint_;
  std::vector<std::string> agent_ids_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  std::string global_frame_;  ///< @brief The global frame for the costmap
  bool rolling_window_;
  std::string tf_prefix_;
  float filter_radius_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace nav2_costmap_2d

#endif  // SOCIAL_NAVIGATION_PLUGINS__PEOPLE_FILTER_LAYER_HPP_
