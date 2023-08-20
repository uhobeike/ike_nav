// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef IKE_NAV_SERVER__IKE_NAV_SERVER_HPP_
#define IKE_NAV_SERVER__IKE_NAV_SERVER_HPP_

#include <nav2_util/robot_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "ike_nav_msgs/action/navigate_to_goal.hpp"
#include "ike_nav_msgs/srv/get_path.hpp"
#include "ike_nav_msgs/srv/get_twist.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using NavigateToGoal = ike_nav_msgs::action::NavigateToGoal;
using GoalHandleNavigateToGoal = rclcpp_action::ServerGoalHandle<NavigateToGoal>;

namespace ike_nav
{

class IkeNavServer : public rclcpp::Node
{
public:
  explicit IkeNavServer(const rclcpp::NodeOptions & options);

protected:
  void initTf();
  void initPublisher();
  void initActionServer();
  void initServiceClient();

  void asyncGetPath(geometry_msgs::msg::PoseStamped start, geometry_msgs::msg::PoseStamped goal);
  void asyncGetTwist(
    const geometry_msgs::msg::PoseStamped & robot_pose, const nav_msgs::msg::Path & path);

  void getMapFrameRobotPose(geometry_msgs::msg::PoseStamped & map_frame_robot_pose);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NavigateToGoal::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToGoal> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleNavigateToGoal> goal_handle);

  void execute(const std::shared_ptr<GoalHandleNavigateToGoal> goal_handle);
  bool checkGoalReached(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
    float & distance_remaining);

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp_action::Server<NavigateToGoal>::SharedPtr navigate_to_goal_action_server_;
  rclcpp::Client<ike_nav_msgs::srv::GetPath>::SharedPtr get_path_client_;
  rclcpp::Client<ike_nav_msgs::srv::GetTwist>::SharedPtr get_twist_client_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::msg::PoseStamped start_;
  nav_msgs::msg::Path path_;
  geometry_msgs::msg::Twist twist_;

  bool get_robot_pose_;
};

}  // namespace ike_nav

#endif  // IKE_NAV_SERVER__IKE_NAV_SERVER_HPP_
