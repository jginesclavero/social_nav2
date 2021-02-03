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
#include "rclcpp/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "social_nav2_goal_updaters/social_goal_updater.hpp"

#define HZ 5

using namespace std::chrono_literals;

namespace social_nav2_actions
{

class HriGoalUpdater : public social_nav2_actions::SocialGoalUpdater
{
public:
  HriGoalUpdater(const std::string & node_name)
  : social_nav2_actions::SocialGoalUpdater(node_name)
  { 
    initParams();
  }

  void getSocialPosesFromAgent(
    std::string id,
    std::vector<geometry_msgs::msg::Pose> & poses)
  {
    tf2::Transform global2agent_tf2;
    if (!getTF("map", id, global2agent_tf2)) {return;}
    
    tf2::Vector3 p1(
      params_map["intimate_z_radius"] + params_map["robot_radius"],
      0.0,
      0.0);
    tf2::Vector3 p2(
      params_map["intimate_z_radius"] + 2 * params_map["robot_radius"],
      0.0,
      0.0);
    tf2::Vector3 p3(
      params_map["personal_z_radius"],
      0.0,
      0.0);
    std::vector<tf2::Vector3> v_p {p1, p2, p3};
    for (auto p : v_p) {
      auto r = global2agent_tf2.getRotation();
      tf2::Matrix3x3 mat(tf2::Quaternion(r.getX(), r.getY(), r.getZ(), r.getW()));
      double yaw, pitch, roll;
      mat.getEulerYPR(yaw, pitch, roll);
      r.setRPY(roll, pitch, yaw - M_PI);
      poses.push_back(tf2ToPose(global2agent_tf2 * p, r));
    }
  }

  bool getBestPose(
    std::vector<geometry_msgs::msg::Pose> & poses,
    geometry_msgs::msg::Pose & best_pose)
  {
    unsigned int mx, my;
    try {
      // Check if the transform is available
      auto costmap = costmap_sub_->getCostmap();
      for (auto pos : poses) {
        if (costmap->worldToMap(pos.position.x, pos.position.y, mx, my)) {
          if (costmap->getCost(mx, my) < 20) {
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

  void step() 
  {
    if (get_current_state().label() == "active") {
      if (agent_id_ != "") {
        if (!action_set) {
          setSocialAction("approaching");
          action_set = true;
        }
        poses_.clear();
        getSocialPosesFromAgent(agent_id_, poses_);
        geometry_msgs::msg::Pose approach_p;
        if (poses_.size() == 0 || !getBestPose(poses_, approach_p)) {
          return;
        }
        updateGoal(approach_p);
      } else {
        RCLCPP_WARN(get_logger(), "Agent_id not initialized, please call set_agent_id srv");
      }
    }
  }

  std::vector<geometry_msgs::msg::Pose> poses_;
  bool action_set;
};

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<social_nav2_actions::HriGoalUpdater>("hri_goal_updater_node");

  rclcpp::Rate loop_rate(HZ);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node->get_node_base_interface());
    node->step();
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  exit(EXIT_SUCCESS);
}
