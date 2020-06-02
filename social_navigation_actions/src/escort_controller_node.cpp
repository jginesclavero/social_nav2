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

#include <memory>

#include "plansys2_msgs/action/execute_action.hpp"

#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

class EscortController : public rclcpp::Node
{
public:
  EscortController()
  : rclcpp::Node("escorting_controller"), state_(STARTING)
  {
  }

  void init()
  {
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>(shared_from_this());
    executor_client_ = std::make_shared<plansys2::ExecutorClient>(shared_from_this());
    executor_state_client_ = shared_from_this()->create_client<lifecycle_msgs::srv::GetState>(
      "/executor/get_state");
    knowledge_ready = false;
  }

  void init_knowledge()
  {
    bool is_server_ready;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for executor...");
      is_server_ready =
        executor_state_client_->wait_for_service(std::chrono::seconds(5));
    } while (!is_server_ready);
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto result = executor_state_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(shared_from_this(), result ) ==
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      if (result.get()->current_state.label == "active")
      {
        problem_expert_->addInstance(plansys2::Instance{"leia", "robot"});
        problem_expert_->addInstance(plansys2::Instance{"agent_1", "agent_id"});
        knowledge_ready = true;
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to call service executor/get_state");
    }
  }

  void step()
  {
    if(!knowledge_ready) {
      init_knowledge();
    } else {
      switch (state_) {
      case STARTING:
        if (executor_client_) {
          // Set the goal for next state, and execute plan
          problem_expert_->setGoal(plansys2::Goal("(and(accompanied agent_1))"));

          if (executor_client_->executePlan()) {
            state_ = ESCORTING;
          }
        }
        break;
      case ESCORTING:
        {
          //RCLCPP_INFO(get_logger(), "ESCORTING STATE");
        }
        break;
      }
    }
  }

private:
  typedef enum {STARTING, ESCORTING} StateType;
  StateType state_;
  bool knowledge_ready;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> executor_state_client_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EscortController>();

  node->init();

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
