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

#ifndef social_nav2_PLUGINS__PEOPLE_FILTER_LAYER_HPP_
#define social_nav2_PLUGINS__PEOPLE_FILTER_LAYER_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

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

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "social_nav2_plugins/geometry/geometry.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

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
  virtual bool isClearable() {return true;}
  
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

#endif  // social_nav2_PLUGINS__PEOPLE_FILTER_LAYER_HPP_
