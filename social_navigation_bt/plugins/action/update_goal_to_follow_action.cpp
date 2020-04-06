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

#ifndef SOCIAL_NAVIGATION_BT__UPDATE_GOAL_TO_FOLLOW_ACTION_HPP_
#define SOCIAL_NAVIGATION_BT__UPDATE_GOAL_TO_FOLLOW_ACTION_HPP_

#include <memory>
#include <string>
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


namespace nav2_behavior_tree
{

class UpdateGoalToFollow : public BT::CoroActionNode
{
public:
  UpdateGoalToFollow(
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::CoroActionNode(action_name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node_->get_node_base_interface(),
      node_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  BT::NodeStatus tick() override
  {
    RCLCPP_INFO(node_->get_logger(), "Getting new pose from agent_test");
    geometry_msgs::msg::PoseStamped pose;
    geometry_msgs::msg::TransformStamped global2agent;

    try 
    {
      // Check if the transform is available
      global2agent = tf_buffer_->lookupTransform("map", "agent_1", tf2::TimePointZero);
    } 
    catch (tf2::TransformException &e) 
    {
      RCLCPP_WARN(node_->get_logger(), "%s", e.what());
      return BT::NodeStatus::FAILURE;
    }
    tf2::Transform global2agent_tf2;
    tf2::impl::Converter<true, false>::convert(global2agent.transform, global2agent_tf2);

    pose.header = global2agent.header;
    pose.pose.orientation = global2agent.transform.rotation;
    pose.pose.position.x = global2agent.transform.translation.x;
    pose.pose.position.y = global2agent.transform.translation.y;

    setOutput("goal", pose);
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {    
    BT::PortsList basic = {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to")
    };
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
factory.registerNodeType<nav2_behavior_tree::UpdateGoalToFollow>(
    "UpdateGoalToFollow");
}

#endif  // SOCIAL_NAVIGATION_BT__UPDATE_GOAL_TO_FOLLOW_ACTION_HPP_
