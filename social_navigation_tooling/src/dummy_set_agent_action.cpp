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
/* Author: jginesclavero jonatan.gines@urjc.es */

/* Mantainer: jginesclavero jonatan.gines@urjc.es */

#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <math.h>
#include "social_navigation_msgs/msg/set_human_action.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;
using SetHumanAction = social_navigation_msgs::msg::SetHumanAction;

class SetAgentAction : public rclcpp::Node
{
public:
  SetAgentAction(const std::string & name) : Node(name)
  {
    action_pub_ =
      create_publisher<SetHumanAction>(
      "/social_navigation/set_agent_action",
      rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable());
  }

  void step()
  {
    auto message = SetHumanAction();
    message.agent_id = "agent_1";
    message.action = "escorting";
    action_pub_->publish(message);

    message.agent_id = "agent_2";
    message.action = "following";
    action_pub_->publish(message);
  }

private:

  rclcpp::Publisher<SetHumanAction>::SharedPtr action_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto agent_action_node = 
    std::make_shared<SetAgentAction>("dummy_set_agent_action_node");
  agent_action_node->step();
  rclcpp::spin(agent_action_node);
  rclcpp::shutdown();
  return 0;
}