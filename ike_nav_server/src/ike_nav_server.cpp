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
  initServiceClient();
  initLoopTimer();

  start_.pose.position.x = 8.1;
  start_.pose.position.y = 11.1;

  goal_.pose.position.x = 11.6;
  goal_.pose.position.y = 8.7;
}

void IkeNavServer::initTf()
{
  tf_buffer_.reset();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  // tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void IkeNavServer::initServiceClient()
{
  get_path_client_ = create_client<ike_nav_msgs::srv::GetPath>("get_path");
  get_twist_client_ = create_client<ike_nav_msgs::srv::GetTwist>("get_twist");
}

void IkeNavServer::initLoopTimer()
{
  loop_timer_ = create_wall_timer(100ms, std::bind(&IkeNavServer::loop, this));
}

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
  };
  auto future_result = get_twist_client_->async_send_request(request, response_received_callback);
}

void IkeNavServer::getMapFrameRobotPose(geometry_msgs::msg::PoseStamped & map_frame_robot_pose)
{
  geometry_msgs::msg::PoseStamped pose;
  if (nav2_util::getCurrentPose(pose, *tf_buffer_)) map_frame_robot_pose = pose;
}

void IkeNavServer::loop()
{
  std::chrono::system_clock::time_point start, end;
  std::time_t time_stamp;

  start = std::chrono::system_clock::now();

  getMapFrameRobotPose(start_);
  asyncGetPath(start_, goal_);
  if (path_.poses.size() > 0) asyncGetTwist(start_, path_);

  end = std::chrono::system_clock::now();

  auto time = end - start;

  time_stamp = std::chrono::system_clock::to_time_t(start);
  std::cout << std::ctime(&time_stamp);

  auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(time).count();
  RCLCPP_INFO(this->get_logger(), "exe_time: %ld[ms]", msec);
}

}  // namespace ike_nav

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ike_nav::IkeNavServer)
