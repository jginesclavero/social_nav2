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

#include <math.h>
#include <utility>
#include <vector>
#include <memory>
#include <string>
#include <map>

#include "plansys2_msgs/action/execute_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_costmap_2d/costmap_layer.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

using GetParameters = rcl_interfaces::srv::GetParameters;
using LifecycleNodeInterface = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

class ScortAction : public plansys2::ActionExecutorClient
{
public:
  ScortAction()
  : plansys2::ActionExecutorClient("escort")
  {
    using namespace std::placeholders;
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(),
      get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    set_rate(1.5);
  }

  LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    costmap_sub_ =
      std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
      shared_from_this(), "global_costmap/costmap_raw");
    params_client_ = shared_from_this()->create_client<GetParameters>(
      "/global_costmap/global_costmap/get_parameters");
    private_node_ = rclcpp::Node::make_shared("social_layer_sub");
    action_pub_ =
      private_node_->create_publisher<diagnostic_msgs::msg::KeyValue>(
      "social_navigation/set_agent_action",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    bool is_server_ready = false;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for params...");
      is_server_ready =
        params_client_->wait_for_service(std::chrono::seconds(10));
    } while (!is_server_ready);

    auto request = std::make_shared<GetParameters::Request>();
    request->names.push_back("robot_radius");
    request->names.push_back("social_layer.intimate_z_radius");

    params_future_ = params_client_->async_send_request(request);
    params_handle_ = params_future_.get();

    params_map.insert(std::pair<std::string, float>(
        "robot_radius",
        params_handle_->values[0].double_value));
    params_map.insert(std::pair<std::string, float>(
        "intimate_z_radius",
        params_handle_->values[1].double_value));
    agent_id_ = getArguments()[1];  // The id is in the 3rd argument of the action
    RCLCPP_INFO(get_logger(), "Agent id [%s]", agent_id_.c_str());
    navigation_action_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(shared_from_this(),
        "navigate_to_pose");
    bool is_action_server_ready = false;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");
      is_action_server_ready =
        navigation_action_client_->wait_for_action_server(std::chrono::seconds(10));
    } while (!is_action_server_ready);

    RCLCPP_INFO(get_logger(), "Navigation action server ready");
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  bool getAgentTF(std::string id, tf2::Transform & tf)
  {
    geometry_msgs::msg::TransformStamped global2agent;
    try {
      // Check if the transform is available
      global2agent = tf_buffer_->lookupTransform("map", id, tf2::TimePointZero);
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "%s", e.what());
      return false;
    }
    tf2::impl::Converter<true, false>::convert(global2agent.transform, tf);
    return true;
  }

  geometry_msgs::msg::Pose tf2ToPose(tf2::Vector3 position, tf2::Quaternion rotation)
  {
    geometry_msgs::msg::Pose output;
    output.position.x = position.getX();
    output.position.y = position.getY();
    output.position.z = position.getZ();

    output.orientation.x = rotation.getX();
    output.orientation.y = rotation.getY();
    output.orientation.z = rotation.getZ();
    output.orientation.w = rotation.getW();
    return output;
  }

  void getNavEscortPosesFromAgent(
    std::string id,
    std::vector<geometry_msgs::msg::Pose> & poses)
  {
    tf2::Transform global2agent_tf2;
    auto r = global2agent_tf2.getRotation();
    if (!getAgentTF(id, global2agent_tf2)) {return;}

    tf2::Vector3 p1(
      params_map["robot_radius"],
      -params_map["intimate_z_radius"] - params_map["robot_radius"],
      0.0);
    tf2::Vector3 p2(
      params_map["robot_radius"],
      -params_map["intimate_z_radius"],
      0.0);
    tf2::Vector3 p3(
      params_map["robot_radius"],
      -params_map["intimate_z_radius"] - 2 * params_map["robot_radius"],
      0.0);
    tf2::Vector3 p4(
      -params_map["intimate_z_radius"] - params_map["robot_radius"],
      0.0,
      0.0);
    std::vector<tf2::Vector3> v_p {p1, p2, p3, p4};
    for (auto p : v_p) {
      poses.push_back(tf2ToPose(global2agent_tf2 * p, global2agent_tf2.getRotation()));
    }
  }

  bool getBetterPose(
    std::vector<geometry_msgs::msg::Pose> & poses,
    geometry_msgs::msg::Pose & best_pose)
  {
    unsigned int mx, my, i = 0;
    try {
      // Check if the transform is available
      auto costmap = costmap_sub_->getCostmap();
      for (auto pos : poses) {
        ++i;
        if (costmap->worldToMap(pos.position.x, pos.position.y, mx, my)) {
          if (costmap->getCost(mx, my) < 20 || (
              i == poses_.size() &&
              costmap->getCost(mx, my) < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
          {
            best_pose = pos;
            return true;
          }
        }
      }
    } catch (std::runtime_error & e) {
      RCLCPP_WARN(get_logger(), "%s", e.what());
    }
    return false;
  }

  void actionStep()
  {
    if (!action_setted && get_current_state().label() == "active") {
      auto message = diagnostic_msgs::msg::KeyValue();
      message.key = agent_id_;
      message.value = "escorting";
      action_pub_->publish(message);
      action_setted = true;
    }

    poses_.clear();
    getNavEscortPosesFromAgent(agent_id_, poses_);
    geometry_msgs::msg::Pose escort_p;
    if (poses_.size() == 0 || !getBetterPose(poses_, escort_p)) {
      return;
    }
    navigation_goal_.pose.pose = escort_p;
    // RCLCPP_INFO(get_logger(), "P [%f %f]", escort_p.position.x, escort_p.position.y);
    future_navigation_goal_handle_ =
      navigation_action_client_->async_send_goal(navigation_goal_);
    navigation_goal_handle_ = future_navigation_goal_handle_.get();
    if (!navigation_goal_handle_) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
      return;
    }
    rclcpp::spin_some(private_node_);
  }

  bool isFinished()
  {
    /* if (getFeedback()->progress >= 100.0) {
      // Check result of navigation
      return true;
    } else {
      return false;
    }*/
  }

  using NavigationGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  rclcpp::Node::SharedPtr private_node_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
  std::shared_future<NavigationGoalHandle::SharedPtr> future_navigation_goal_handle_;
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  std::shared_future<std::shared_ptr<rcl_interfaces::srv::GetParameters_Response>> params_future_;
  rcl_interfaces::srv::GetParameters_Response::SharedPtr params_handle_;
  std::shared_ptr<rclcpp::Client<GetParameters>> params_client_;
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::map<std::string, float> params_map;
  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::KeyValue>::SharedPtr action_pub_;
  std::string agent_id_;
  std::vector<geometry_msgs::msg::Pose> poses_;
  bool action_setted;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScortAction>();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
