// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef IKE_WAYPOINT_FOLLOWER__IKE_WAYPOINT_FOLLOWER_HPP_
#define IKE_WAYPOINT_FOLLOWER__IKE_WAYPOINT_FOLLOWER_HPP_

#include "ike_waypoint_follower_parameter/ike_waypoint_follower_parameter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "ike_nav_msgs/action/navigate_to_goal.hpp"
#include "ike_nav_msgs/msg/waypoints.hpp"

using NavigateToGoal = ike_nav_msgs::action::NavigateToGoal;
using GoalHandleNavigateToGoal = rclcpp_action::ServerGoalHandle<NavigateToGoal>;

namespace ike_nav
{
class IkeWaypointFollower : public rclcpp::Node
{
public:
  explicit IkeWaypointFollower(const rclcpp::NodeOptions & options);

protected:
  void getParam();

  void initActionClient();
  void initTimer();

  void readWaypointYaml();

  void loop();

private:
  rclcpp_action::Client<NavigateToGoal>::SharedPtr navigate_to_goal_action_client_;

  rclcpp::TimerBase::SharedPtr loop_timer_;

  std::shared_ptr<ike_waypoint_follower::ParamListener> param_listener_;
  ike_waypoint_follower::Params params_;

  std::string waypoint_yaml_path_;
  ike_nav_msgs::msg::Waypoints waypoints_;
};

}  // namespace ike_nav

#endif  // IKE_WAYPOINT_FOLLOWER__IKE_WAYPOINT_FOLLOWER_HPP_
