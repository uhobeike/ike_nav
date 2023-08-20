// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "ike_nav_server/ike_nav_server.hpp"

#include <memory>
#include <utility>

using namespace std::chrono_literals;

namespace ike_nav
{

IkeNavServer::IkeNavServer(const rclcpp::NodeOptions & options) : Node("ike_nav_server", options)
{
  initTf();
  initPublisher();
  initServiceClient();
  initActionServer();
}

void IkeNavServer::initTf()
{
  tf_buffer_.reset();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  // tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void IkeNavServer::initPublisher()
{
  cmd_vel_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(1).reliable());
};

void IkeNavServer::initActionServer()
{
  using namespace std::placeholders;

  navigate_to_goal_action_server_ = rclcpp_action::create_server<NavigateToGoal>(
    this->get_node_base_interface(), this->get_node_clock_interface(),
    this->get_node_logging_interface(), this->get_node_waitables_interface(), "navigate_to_goal",
    std::bind(&IkeNavServer::handle_goal, this, _1, _2),
    std::bind(&IkeNavServer::handle_cancel, this, _1),
    std::bind(&IkeNavServer::handle_accepted, this, _1));
}

void IkeNavServer::initServiceClient()
{
  get_path_client_ = create_client<ike_nav_msgs::srv::GetPath>("get_path");
  get_twist_client_ = create_client<ike_nav_msgs::srv::GetTwist>("get_twist");
}

rclcpp_action::GoalResponse IkeNavServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  [[maybe_unused]] std::shared_ptr<const NavigateToGoal::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received navigate_to_goal request");
  (void)uuid;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
};

rclcpp_action::CancelResponse IkeNavServer::handle_cancel(
  const std::shared_ptr<GoalHandleNavigateToGoal> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received navigate_to_goal request to cancel");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
};

void IkeNavServer::handle_accepted(const std::shared_ptr<GoalHandleNavigateToGoal> goal_handle)
{
  using namespace std::placeholders;

  std::thread{std::bind(&IkeNavServer::execute, this, _1), goal_handle}.detach();
};

void IkeNavServer::asyncGetPath(
  geometry_msgs::msg::PoseStamped start, geometry_msgs::msg::PoseStamped goal)
{
  while (!get_path_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  auto request = std::make_shared<ike_nav_msgs::srv::GetPath::Request>();
  request->start = start;
  request->goal = goal;

  using ServiceResponseFuture = rclcpp::Client<ike_nav_msgs::srv::GetPath>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
    auto result = future.get();
    path_ = result->path;
  };
  auto future_result = get_path_client_->async_send_request(request, response_received_callback);
}
void IkeNavServer::asyncGetTwist(
  const geometry_msgs::msg::PoseStamped & robot_pose, const nav_msgs::msg::Path & path)
{
  while (!get_path_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  auto request = std::make_shared<ike_nav_msgs::srv::GetTwist::Request>();
  request->robot_pose = robot_pose;
  request->path = path;

  using ServiceResponseFuture = rclcpp::Client<ike_nav_msgs::srv::GetTwist>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
    auto result = future.get();
    twist_ = result->twist.twist;
    cmd_vel_pub_->publish(twist_);
  };
  auto future_result = get_twist_client_->async_send_request(request, response_received_callback);
}

void IkeNavServer::getMapFrameRobotPose(geometry_msgs::msg::PoseStamped & map_frame_robot_pose)
{
  geometry_msgs::msg::PoseStamped pose;
  if (nav2_util::getCurrentPose(pose, *tf_buffer_)) {
    map_frame_robot_pose = pose;
    get_robot_pose_ = true;
  }
}

bool IkeNavServer::checkGoalReached(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
  float & distance_remaining)
{
  distance_remaining = std::hypot(
    start.pose.position.x - goal.pose.position.x, start.pose.position.y - goal.pose.position.y);

  if (distance_remaining < 0.5) return true;

  return false;
}

void IkeNavServer::execute(const std::shared_ptr<GoalHandleNavigateToGoal> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing navigate_to_goal");

  rclcpp::Rate loop_rate(5);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<NavigateToGoal::Feedback>();
  auto & distance_remaining = feedback->distance_remaining;
  auto result = std::make_shared<NavigateToGoal::Result>();

  while (rclcpp::ok()) {
    std::chrono::system_clock::time_point start, end;
    std::time_t time_stamp;

    start = std::chrono::system_clock::now();

    getMapFrameRobotPose(start_);
    if (get_robot_pose_) asyncGetPath(start_, goal->pose);
    if (path_.poses.size() > 0) asyncGetTwist(start_, path_);

    end = std::chrono::system_clock::now();

    auto time = end - start;

    time_stamp = std::chrono::system_clock::to_time_t(start);
    std::cout << std::ctime(&time_stamp);

    auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(time).count();
    RCLCPP_INFO(this->get_logger(), "exe_time: %ld[ms]", msec);

    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "navigate_to_goal Canceled");
      return;
    }

    if (checkGoalReached(start_, goal->pose, distance_remaining)) {
      result->goal_reached = true;
      RCLCPP_INFO(this->get_logger(), "Goal Reached");
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
      break;
    }

    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }

  if (result->goal_reached) goal_handle->succeed(result);

  RCLCPP_INFO(this->get_logger(), "Done navigate_to_goal");
}

}  // namespace ike_nav

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ike_nav::IkeNavServer)
