// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef IKE_WAYPOINT_FOLLOWER__IKE_WAYPOINT_FOLLOWER_HPP_
#define IKE_WAYPOINT_FOLLOWER__IKE_WAYPOINT_FOLLOWER_HPP_

#include "ike_waypoint_follower_parameter/ike_waypoint_follower_parameter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "ike_nav_msgs/action/navigate_to_goal.hpp"
#include "ike_nav_msgs/msg/waypoints.hpp"
#include "ike_nav_msgs/srv/load_waypoint_yaml.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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

  void initTf();
  void initPublisher();
  void initServiceServer();
  void initActionClient();
  void initTimer();

  void readWaypointYaml();
  void getMapFrameRobotPose(geometry_msgs::msg::PoseStamped & map_frame_robot_pose);
  bool isInsideWaypointArea(
    const geometry_msgs::msg::Pose & robot_pose, const ike_nav_msgs::msg::Waypoint & waypoint);
  void sendGoal(const geometry_msgs::msg::Pose & goal);
  void cancelGoal();

  void loop();

private:
  // clang-format off
  rclcpp::Publisher<ike_nav_msgs::msg::Waypoints>::SharedPtr waypoints_pub_;
  rclcpp::Service<ike_nav_msgs::srv::LoadWaypointYaml>::SharedPtr load_waypoint_yaml_service_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_waypoint_follower_service_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_waypoint_follower_service_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_waypoint_follower_service_server_;
  rclcpp_action::Client<NavigateToGoal>::SharedPtr navigate_to_goal_action_client_;
  // clang-format on

  rclcpp::TimerBase::SharedPtr loop_timer_;

  std::shared_ptr<ike_waypoint_follower::ParamListener> param_listener_;
  ike_waypoint_follower::Params params_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string waypoint_yaml_path_;
  ike_nav_msgs::msg::Waypoints waypoints_;
  double waypoint_radius_;

  u_int32_t waypoint_id_;
  geometry_msgs::msg::PoseStamped robot_pose_;

  bool get_robot_pose_;
};

}  // namespace ike_nav

#endif  // IKE_WAYPOINT_FOLLOWER__IKE_WAYPOINT_FOLLOWER_HPP_
