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
#include "social_nav2_goal_updaters/social_goal_updater.hpp"

namespace social_nav2_actions
{
  SocialGoalUpdater::SocialGoalUpdater(const std::string & node_name)
  : rclcpp_cascade_lifecycle::CascadeLifecycleNode(node_name), 
    tf2_buffer_(get_clock()),
    tf2_listener_(tf2_buffer_, true)
  {
    initParams();
    node_name_ = node_name;
  }

  CallbackReturnT SocialGoalUpdater::on_configure(const rclcpp_lifecycle::State & state) {
    costmap_sub_ =
      std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
      shared_from_this(), "global_costmap/costmap_raw");
    rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
    goal_update_pub_ =
      std::make_shared<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>>(
        get_node_base_interface().get(), 
        goal_update_topic_, 
        rclcpp::QoS(1),
        options);
    action_pub_ =
      std::make_shared<rclcpp_lifecycle::LifecyclePublisher<SetHumanAction>>(
        get_node_base_interface().get(), 
        std::string("/social_nav2/set_agent_action"), 
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
        options);
    chrono_pub_ =
      std::make_shared<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>>(
        get_node_base_interface().get(), 
        std::string("time_B"), 
        rclcpp::QoS(1),
        options);

    // add_publisher_handle(goal_update_pub_);
    // add_publisher_handle(action_pub_);
    // add_publisher_handle(chrono_pub_);

    goal_update_pub_->on_activate();
    action_pub_->on_activate();
    chrono_pub_->on_activate();
    private_node_ = rclcpp::Node::make_shared("hri_goal_updater_get_params");
    params_client_ = private_node_->create_client<GetParameters>(
      "/global_costmap/global_costmap/get_parameters");
    goal_updater_params_service_ = create_service<SetParameters>(
      node_name_ + "/set_parameters", std::bind(&SocialGoalUpdater::setParameters, this, _1, _2));
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  CallbackReturnT SocialGoalUpdater::on_activate(const rclcpp_lifecycle::State & state) 
  {
    getParams();
    RCLCPP_INFO(get_logger(), "Lifecyclenode ready");
    std_msgs::msg::Float64 msg;
    msg.data = now().seconds();
    chrono_pub_->publish(msg);
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  void SocialGoalUpdater::getParams()
  {
    bool is_server_ready = false;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for global_costmap params srv...");
      is_server_ready =
        params_client_->wait_for_service(std::chrono::seconds(5));
    } while (!is_server_ready);

    auto request = std::make_shared<GetParameters::Request>();
    request->names.push_back("robot_radius");
    request->names.push_back("social_layer.intimate_z_radius");
    request->names.push_back("social_layer.personal_z_radius");

    params_future_ = params_client_->async_send_request(request).future.share();

    if (rclcpp::spin_until_future_complete(private_node_, params_future_) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      params_handle_ = params_future_.get();
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to call service global_costmap params srv");
    }

    params_map.insert(std::pair<std::string, float>(
        "robot_radius",
        params_handle_->values[0].double_value));
    params_map.insert(std::pair<std::string, float>(
        "intimate_z_radius",
        params_handle_->values[1].double_value));
    params_map.insert(std::pair<std::string, float>(
        "personal_z_radius",
        params_handle_->values[2].double_value));
  }

  bool SocialGoalUpdater::getTF(std::string target, std::string source, tf2::Transform & tf)
  {
    geometry_msgs::msg::TransformStamped global2agent;
    try {
      // Check if the transform is available
      global2agent = tf2_buffer_.lookupTransform(target, source, tf2::TimePointZero);
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "%s", e.what());
      return false;
    }
    tf2::impl::Converter<true, false>::convert(global2agent.transform, tf);
    return true;
  }

  geometry_msgs::msg::Pose SocialGoalUpdater::tf2ToPose(
    tf2::Vector3 position, 
    tf2::Quaternion rotation)
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

  void SocialGoalUpdater::updateGoal(const geometry_msgs::msg::Pose & goal)
  {
    geometry_msgs::msg::PoseStamped pose_stamped;

    pose_stamped.header.frame_id = static_frame_;
    pose_stamped.header.stamp = now();
    pose_stamped.pose = goal;

    goal_update_pub_->publish(pose_stamped);

    RCLCPP_DEBUG(get_logger(), "Update goal!");
  }

  void SocialGoalUpdater::setParameters(
    const std::shared_ptr<SetParameters::Request> request,
    std::shared_ptr<SetParameters::Response> response)
  {
    for (auto param : request->parameters) {
      if (param.name == "agent_id") {
        agent_id_ = param.value.string_value;
        RCLCPP_INFO(get_logger(), "Agent id was set [%s]", agent_id_.c_str());
      } else if (param.name == "social_layer.intimate_z_radius") {
        params_map["intimate_z_radius"] = param.value.double_value;
      } else if (param.name == "social_layer.personal_z_radius") {
        params_map["personal_z_radius"] = param.value.double_value;
      }
    }
  }
  void SocialGoalUpdater::setSocialAction(std::string action) 
  {
    auto message = SetHumanAction();
    message.agent_id = agent_id_;
    message.action = action;
    action_pub_->publish(message);
  }

  void SocialGoalUpdater::initParams()
  {
    agent_id_ = "";
    static_frame_ = "map";
    goal_update_topic_ = "/goal_update";
  }
}