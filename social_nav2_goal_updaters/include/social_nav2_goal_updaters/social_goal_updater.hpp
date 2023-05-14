// Copyright 2020
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

/* Author: Jonatan Ginés jonatan.gines@urjc.es */

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "rclcpp/time.hpp"

#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "social_nav2_msgs/msg/set_human_action.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using CallbackReturnT =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using GetParameters = rcl_interfaces::srv::GetParameters;
using SetParameters = rcl_interfaces::srv::SetParameters;
using SetHumanAction = social_nav2_msgs::msg::SetHumanAction;
using LifecycleNodeInterface = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

namespace social_nav2_actions
{

class SocialGoalUpdater : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  SocialGoalUpdater(const std::string & node_name);

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  virtual void step() { }

  bool getTF(std::string target, std::string source, tf2::Transform & tf);

  geometry_msgs::msg::Pose tf2ToPose(
    tf2::Vector3 position, tf2::Quaternion rotation);

  void updateGoal(const geometry_msgs::msg::Pose & goal);
  void setParameters(const std::shared_ptr<SetParameters::Request> request,
    std::shared_ptr<SetParameters::Response> response);
  void initParams();
  void getParams();
  void setSocialAction(std::string action);

  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<SetHumanAction>> action_pub_;
  std::shared_ptr<
    rclcpp_lifecycle::LifecyclePublisher<
      geometry_msgs::msg::PoseStamped>> goal_update_pub_;
  std::shared_ptr<
    rclcpp_lifecycle::LifecyclePublisher<
      std_msgs::msg::Float64>> chrono_pub_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Node::SharedPtr private_node_;
  std::shared_future<
    std::shared_ptr<rcl_interfaces::srv::GetParameters_Response>> params_future_;
  rcl_interfaces::srv::GetParameters_Response::SharedPtr params_handle_;
  std::shared_ptr<rclcpp::Client<GetParameters>> params_client_;
  std::map<std::string, float> params_map;
  rclcpp::Service<SetParameters>::SharedPtr goal_updater_params_service_;
  std::string agent_id_, goal_update_topic_, static_frame_, node_name_;
  std::vector<geometry_msgs::msg::Pose> poses_;
  bool action_set;
};

}