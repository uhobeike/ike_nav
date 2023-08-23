// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "ike_controller/ike_controller.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

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
  getParam();

  initPublisher();
  initService();
  setMpcParameters();
}

void IkeController::getParam()
{
  this->param_listener_ =
    std::make_shared<ike_controller::ParamListener>(this->get_node_parameters_interface());
  this->params_ = param_listener_->get_params();

  dt_ = this->params_.mpc.delta_time;
  predictive_horizon_num_ = this->params_.mpc.predictive_horizon_num;
  lower_bound_linear_velocity_ = this->params_.mpc.lower_bound_linear_velocity;
  lower_bound_angular_velocity_ = this->params_.mpc.lower_bound_angular_velocity;
  upper_bound_linear_velocity_ = this->params_.mpc.upper_bound_linear_velocity;
  upper_bound_angular_velocity_ = this->params_.mpc.upper_bound_angular_velocity;
  max_num_iterations_ = this->params_.mpc.max_num_iterations;
}

void IkeController::initPublisher()
{
  predictive_horizon_pub_ =
    this->create_publisher<nav_msgs::msg::Path>("predictive_horizon", rclcpp::QoS(1).reliable());
}

void IkeController::initService()
{
  auto get_twist = [&](
                     const std::shared_ptr<rmw_request_id_t> request_header,
                     const std::shared_ptr<ike_nav_msgs::srv::GetTwist_Request> request,
                     std::shared_ptr<ike_nav_msgs::srv::GetTwist_Response> response) -> void {
    (void)request_header;
    RCLCPP_INFO(this->get_logger(), "IkeController Model Predictive Control start");
    response->twist = ModelPredictiveControl(request->robot_pose, request->path);
    RCLCPP_INFO(this->get_logger(), "IkeController Model Predictive Control done");
  };
  get_twist_srv_ = create_service<ike_nav_msgs::srv::GetTwist>("get_twist", get_twist);
}

void IkeController::setMpcParameters()
{
  // MPC Parameters
  constexpr double dt = 1.0;
  int predictive_horizon_num = 10;
  constexpr double lower_bound_linear_velocity = 0.0;
  constexpr double lower_bound_angular_velocity = -M_PI;
  constexpr double upper_bound_linear_velocity = 1.0;
  constexpr double upper_bound_angular_velocity = M_PI;
  constexpr int max_num_iterations = 100;

  dt_ = dt;
  predictive_horizon_num_ = predictive_horizon_num;
  lower_bound_linear_velocity_ = lower_bound_linear_velocity;
  lower_bound_angular_velocity_ = lower_bound_angular_velocity;
  upper_bound_linear_velocity_ = upper_bound_linear_velocity;
  upper_bound_angular_velocity_ = upper_bound_angular_velocity;
  max_num_iterations_ = max_num_iterations;
}

geometry_msgs::msg::TwistStamped IkeController::ModelPredictiveControl(
  const geometry_msgs::msg::PoseStamped & robot_pose, nav_msgs::msg::Path & path)
{
  RCLCPP_INFO(this->get_logger(), "ModelPredictiveControl start.");

  nav_msgs::msg::Path skip_path;

  int interval = 2;
  for (size_t i = 0; i < path.poses.size(); i += interval) {
    skip_path.poses.push_back(path.poses[i]);
  }
  path = skip_path;

  std::tuple<double, double, double> robot_pose_tuple = {
    robot_pose.pose.position.x, robot_pose.pose.position.y,
    tf2::getYaw(robot_pose.pose.orientation)};

  auto path_xy = convertPathXY(path);
  auto action = optimize(robot_pose_tuple, path_xy);
  auto predictive_horizon = getPredictiveHorizon(robot_pose_tuple, action);
  publishPredictiveHorizon(predictive_horizon);
  auto twist = convertTwist(action);

  RCLCPP_INFO(this->get_logger(), "ModelPredictiveControl done.");

  return twist;
}

std::pair<std::vector<double>, std::vector<double>> IkeController::convertPathXY(
  const nav_msgs::msg::Path & path)
{
  std::vector<double> path_x, path_y;
  for (const auto & pose_stamped : path.poses) {
    path_x.push_back(pose_stamped.pose.position.x);
    path_y.push_back(pose_stamped.pose.position.y);
  }

  return std::make_pair(path_x, path_y);
}

std::pair<std::vector<double>, std::vector<double>> IkeController::optimize(
  const std::tuple<double, double, double> & robot_pose,
  const std::pair<std::vector<double>, std::vector<double>> & path)
{
  // todo fix
  if (path.first.size() < static_cast<long unsigned int>(predictive_horizon_num_)) {
    predictive_horizon_num_ = path.first.size();
  } else {
    predictive_horizon_num_ = 10;
  }

  auto * cost_function =
    new ceres::DynamicAutoDiffCostFunction<ObjectiveFunction, MAX_PREDICTIVE_HORIZON_NUM>(
      new ObjectiveFunction(
        std::get<0>(robot_pose), std::get<1>(robot_pose), std::get<2>(robot_pose),
        std::get<0>(path), std::get<1>(path), dt_, predictive_horizon_num_));

  cost_function->SetNumResiduals(predictive_horizon_num_);
  cost_function->AddParameterBlock(predictive_horizon_num_);
  cost_function->AddParameterBlock(predictive_horizon_num_);

  std::vector<double> v_in, w_in, v_out, w_out;
  std::vector<std::vector<double> *> vectors = {&v_in, &w_in, &v_out, &w_out};
  for (auto vec : vectors) vec->assign(predictive_horizon_num_, 0.0);

  Problem problem;
  problem.AddResidualBlock(cost_function, nullptr, v_out.data(), w_out.data());
  for (int i = 0; i < predictive_horizon_num_; ++i) {
    problem.SetParameterLowerBound(v_out.data(), i, lower_bound_linear_velocity_);
    problem.SetParameterLowerBound(w_out.data(), i, lower_bound_angular_velocity_);
    problem.SetParameterUpperBound(v_out.data(), i, upper_bound_linear_velocity_);
    problem.SetParameterUpperBound(w_out.data(), i, upper_bound_angular_velocity_);
  }

  Solver::Options options;
  options.max_num_iterations = max_num_iterations_;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;

  Solver::Summary summary;
  Solve(options, &problem, &summary);

  RCLCPP_INFO(this->get_logger(), "%s", summary.BriefReport().c_str());

  return std::make_pair(v_out, w_out);
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>
IkeController::getPredictiveHorizon(
  const std::tuple<double, double, double> & robot_pose,
  const std::pair<std::vector<double>, std::vector<double>> & action)
{
  std::vector<double> predictive_horizon_x, predictive_horizon_y, predictive_horizon_ths;

  predictive_horizon_x.push_back(std::get<0>(robot_pose));
  predictive_horizon_y.push_back(std::get<1>(robot_pose));
  predictive_horizon_ths.push_back(std::get<2>(robot_pose));

  for (int i = 0; i < predictive_horizon_num_; i++) {
    // clang-format off
    double  x =
          predictive_horizon_x[i] + 
            action.first[i] * cos(predictive_horizon_ths[i]) * dt_;
    double  y = 
          predictive_horizon_y[i] +
            action.first[i] * sin(predictive_horizon_ths[i]) * dt_;
    double  th = 
          predictive_horizon_ths[i] + 
            action.second[i] * dt_;
    // clang-format on

    predictive_horizon_x.push_back(x);
    predictive_horizon_y.push_back(y);
    predictive_horizon_ths.push_back(th);
  }

  return std::make_tuple(predictive_horizon_x, predictive_horizon_y, predictive_horizon_ths);
}

void IkeController::publishPredictiveHorizon(
  const std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> &
    predictive_horizon)
{
  auto x_size = std::get<0>(predictive_horizon).size();
  auto y_size = std::get<1>(predictive_horizon).size();
  auto th_size = std::get<2>(predictive_horizon).size();

  if (x_size != y_size || y_size != th_size) {
    RCLCPP_ERROR(this->get_logger(), "Different array sizes.");
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = rclcpp::Time();
  path.poses.resize(x_size);

  for (size_t i = 0; i < x_size; ++i) {
    path.poses[i].pose.position.x = std::get<0>(predictive_horizon)[i];
    path.poses[i].pose.position.y = std::get<1>(predictive_horizon)[i];
    path.poses[i].pose.orientation.w = cos(std::get<2>(predictive_horizon)[i] / 2.0);
    path.poses[i].pose.orientation.z = sin(std::get<2>(predictive_horizon)[i] / 2.0);
  }

  predictive_horizon_pub_->publish(path);
}

geometry_msgs::msg::TwistStamped IkeController::convertTwist(
  const std::pair<std::vector<double>, std::vector<double>> & action)
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = "";
  twist.header.stamp = rclcpp::Time();

  // todo fix
  twist.twist.linear.x = action.first[0] + action.first[1];
  twist.twist.angular.z = action.second[0] + action.second[1];

  return twist;
}

}  // namespace ike_nav

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ike_nav::IkeController)
