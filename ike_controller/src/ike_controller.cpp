// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "ike_controller/ike_controller.hpp"

#include <ceres/ceres.h>
#include <glog/logging.h>

#include <memory>
#include <utility>

using ceres::CostFunction;
using ceres::DynamicAutoDiffCostFunction;
using ceres::Problem;
using ceres::Solve;

using namespace std::chrono_literals;

constexpr int MAX_PREDICTIVE_HORIZON_NUM = 100;

namespace ike_nav
{

IkeController::IkeController(const rclcpp::NodeOptions & options) : Node("ike_controller", options)
{
  initTf();
  initServiceClient();
  initLoopTimer();

  start_.pose.position.x = 8.1;
  start_.pose.position.y = 11.1;

  goal_.pose.position.x = 11.6;
  goal_.pose.position.y = 8.7;
}

void IkeController::initTf()
{
  tf_buffer_.reset();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  // tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void IkeController::initServiceClient()
{
  get_path_client_ = create_client<ike_nav_msgs::srv::GetPath>("get_path");
}

void IkeController::initLoopTimer()
{
  loop_timer_ = create_wall_timer(100ms, std::bind(&IkeController::loop, this));
}

void IkeController::asyncGetPath(
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

void IkeController::getMapFrameRobotPose(geometry_msgs::msg::PoseStamped & map_frame_robot_pose)
{
  geometry_msgs::msg::PoseStamped pose;
  if (nav2_util::getCurrentPose(pose, *tf_buffer_)) map_frame_robot_pose = pose;
}

void IkeController::ModelPredictiveControl()
{
  RCLCPP_INFO(this->get_logger(), "MPC start.");

  constexpr double dt = 1;
  constexpr int predictive_horizon_num = 40;

  constexpr double lower_bound_linear_velocity = 0.0;
  constexpr double lower_bound_angular_velocity = -M_PI;
  constexpr double upper_bound_linear_velocity = 1.0;
  constexpr double upper_bound_angular_velocity = M_PI;

  std::vector<double> path_x;
  std::vector<double> path_y;

  for (const auto & pose_stamped : path_.poses) {
    path_x.push_back(pose_stamped.pose.position.x);
    path_y.push_back(pose_stamped.pose.position.y);
    RCLCPP_INFO(
      this->get_logger(), "pose %f, %f", pose_stamped.pose.position.x,
      pose_stamped.pose.position.y);
  }

  auto * cost_function =
    new ceres::DynamicAutoDiffCostFunction<ObjectiveFunction, MAX_PREDICTIVE_HORIZON_NUM>(
      new ObjectiveFunction(path_x, path_y, dt, predictive_horizon_num));

  cost_function->SetNumResiduals(predictive_horizon_num);
  cost_function->AddParameterBlock(predictive_horizon_num);
  cost_function->AddParameterBlock(predictive_horizon_num);

  std::vector<double> v_in, w_in, v_out, w_out;
  std::vector<std::vector<double> *> vectors = {&v_in, &w_in, &v_out, &w_out};
  for (auto vec : vectors) vec->assign(predictive_horizon_num, 0.0);

  Problem problem;
  problem.AddResidualBlock(cost_function, nullptr, v_out.data(), w_out.data());

  for (int i = 0; i < predictive_horizon_num; ++i) {
    problem.SetParameterLowerBound(v_out.data(), i, lower_bound_linear_velocity);
    problem.SetParameterLowerBound(w_out.data(), i, lower_bound_angular_velocity);
    problem.SetParameterUpperBound(v_out.data(), i, upper_bound_linear_velocity);
    problem.SetParameterUpperBound(w_out.data(), i, upper_bound_angular_velocity);
  }

  Solver::Options options;
  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;

  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";

  RCLCPP_INFO(this->get_logger(), "%s", summary.BriefReport().c_str());

  RCLCPP_INFO(this->get_logger(), "MPC done.");
}

void IkeController::loop()
{
  std::chrono::system_clock::time_point start, end;
  std::time_t time_stamp;

  start = std::chrono::system_clock::now();

  getMapFrameRobotPose(start_);
  asyncGetPath(start_, goal_);

  if (path_.poses.size() != 0) ModelPredictiveControl();

  end = std::chrono::system_clock::now();

  auto time = end - start;

  time_stamp = std::chrono::system_clock::to_time_t(start);
  std::cout << std::ctime(&time_stamp);

  auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(time).count();
  RCLCPP_INFO(this->get_logger(), "exe_time: %ld[ms]", msec);
}

}  // namespace ike_nav

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ike_nav::IkeController)
