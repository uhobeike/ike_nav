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
  getParam();

  initTf();
  initPublisher();
  initSubscription();
  initServiceClient();
  initActionServer();
  initActionClient();
  initTimer();
}

void IkeNavServer::getParam()
{
  this->param_listener_ =
    std::make_shared<ike_nav_server::ParamListener>(this->get_node_parameters_interface());
  this->params_ = param_listener_->get_params();

  ike_nav_server_loop_hz_ = this->params_.ike_nav_server_loop_hz;
  goal_tolerance_xy_ = this->params_.goal_tolerance_xy;
  publish_stop_velocity_ms_ = 1 / this->params_.publish_stop_velocity_hz * 1000.;
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

void IkeNavServer::initSubscription()
{
  auto goal_pose_callback = [this](geometry_msgs::msg::PoseStamped msg) -> void {
    if (!navigate_to_goal_action_client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = NavigateToGoal::Goal();
    goal_msg.pose = msg;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<NavigateToGoal>::SendGoalOptions();
    auto goal_handle_future =
      navigate_to_goal_action_client_->async_send_goal(goal_msg, send_goal_options);
  };
  goal_pose_sub_ =
    this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 1, goal_pose_callback);
}

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

void IkeNavServer::initActionClient()
{
  navigate_to_goal_action_client_ = rclcpp_action::create_client<NavigateToGoal>(
    this->get_node_base_interface(), this->get_node_graph_interface(),
    this->get_node_logging_interface(), this->get_node_waitables_interface(), "navigate_to_goal");
};

void IkeNavServer::initServiceClient()
{
  get_path_client_ = this->create_client<ike_nav_msgs::srv::GetPath>("get_path");
  get_twist_client_ = this->create_client<ike_nav_msgs::srv::GetTwist>("get_twist");
}

void IkeNavServer::initTimer()
{
  stop_velocity_publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds{publish_stop_velocity_ms_},
    [this]() { cmd_vel_pub_->publish(geometry_msgs::msg::Twist()); });
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
  stop_velocity_publish_timer_->cancel();

  using namespace std::placeholders;

  auto thread = std::thread{std::bind(&IkeNavServer::execute, this, _1), goal_handle};
  thread_id_.push_back(thread.get_id());
  thread.detach();
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

  if (distance_remaining < goal_tolerance_xy_) return true;

  return false;
}

bool IkeNavServer::checkShouldExitThisThread()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (thread_id_.size() > 1) {
    if (*thread_id_.begin() == std::this_thread::get_id()) {
      thread_id_.erase(thread_id_.begin());
      return true;
    }
  }

  return false;
}

void IkeNavServer::clearThreadId()
{
  std::lock_guard<std::mutex> lock(mutex_);

  thread_id_.clear();
}

void IkeNavServer::execute(const std::shared_ptr<GoalHandleNavigateToGoal> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing navigate_to_goal");

  rclcpp::Rate loop_rate(ike_nav_server_loop_hz_);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<NavigateToGoal::Feedback>();
  auto & distance_remaining = feedback->distance_remaining;
  auto result = std::make_shared<NavigateToGoal::Result>();
  auto should_exit_this_thread = false;

  while (rclcpp::ok()) {
    std::chrono::system_clock::time_point start, end;

    start = std::chrono::system_clock::now();

    getMapFrameRobotPose(start_);
    if (get_robot_pose_) asyncGetPath(start_, goal->pose);
    if (path_.poses.size() > 0) asyncGetTwist(start_, path_);

    end = std::chrono::system_clock::now();

    auto time = end - start;
    auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(time).count();
    RCLCPP_INFO(this->get_logger(), "exe_time: %ld[ms]", msec);

    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "navigate_to_goal Canceled");
      stop_velocity_publish_timer_->reset();
      return;
    }

    // todo fix related to https://github.com/ros2/ros2/issues/1253
    if (checkShouldExitThisThread()) {
      result->goal_reached = false;
      should_exit_this_thread = true;
      RCLCPP_INFO(this->get_logger(), "Drop this thread as other thread have started up");
      break;
    }

    if (checkGoalReached(start_, goal->pose, distance_remaining)) {
      result->goal_reached = true;
      RCLCPP_INFO(this->get_logger(), "Goal Reached");
      clearThreadId();
      stop_velocity_publish_timer_->reset();
      break;
    }

    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }

  if (result->goal_reached) goal_handle->succeed(result);
  if (should_exit_this_thread) goal_handle->succeed(result);

  RCLCPP_INFO(this->get_logger(), "Done navigate_to_goal");
}

}  // namespace ike_nav

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ike_nav::IkeNavServer)