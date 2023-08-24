// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef IKE_CONTROLLER__IKE_CONTROLLER_HPP_
#define IKE_CONTROLLER__IKE_CONTROLLER_HPP_

#include "ike_controller_parameter/ike_controller_parameter.hpp"

#include <rclcpp/rclcpp.hpp>

#include "ike_nav_msgs/srv/get_twist.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <ceres/ceres.h>

using ceres::Solver;

namespace ike_nav
{
class IkeController : public rclcpp::Node
{
public:
  explicit IkeController(const rclcpp::NodeOptions & options);

protected:
  void getParam();

  void initPublisher();
  void initService();

  geometry_msgs::msg::TwistStamped ModelPredictiveControl(
    const geometry_msgs::msg::PoseStamped & robot_pose, nav_msgs::msg::Path & path);
  std::pair<std::vector<double>, std::vector<double>> convertPathXY(
    const nav_msgs::msg::Path & path);
  std::pair<std::vector<double>, std::vector<double>> optimize(
    const std::tuple<double, double, double> & robot_pose,
    const std::pair<std::vector<double>, std::vector<double>> & path);
  std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> getPredictiveHorizon(
    const std::tuple<double, double, double> & robot_pose,
    const std::pair<std::vector<double>, std::vector<double>> & action);

  void publishPredictiveHorizon(
    const std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> &
      predictive_horizon);

  geometry_msgs::msg::TwistStamped convertTwist(
    const std::pair<std::vector<double>, std::vector<double>> & action);

private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predictive_horizon_pub_;
  rclcpp::Service<ike_nav_msgs::srv::GetTwist>::SharedPtr get_twist_srv_;

  std::shared_ptr<ike_controller::ParamListener> param_listener_;
  ike_controller::Params params_;

  geometry_msgs::msg::PoseStamped start_, goal_;
  nav_msgs::msg::Path path_;

  double dt_;
  int predictive_horizon_num_;
  double lower_bound_linear_velocity_;
  double lower_bound_angular_velocity_;
  double upper_bound_linear_velocity_;
  double upper_bound_angular_velocity_;
  int max_num_iterations_;
};

struct ObjectiveFunction
{
  ObjectiveFunction(
    double init_pose_x, double init_pose_y, double init_pose_th, std::vector<double> path_x,
    std::vector<double> path_y, double dt, int predictive_horizon_num)
  : init_pose_x_(init_pose_x),
    init_pose_y_(init_pose_y),
    init_pose_th_(init_pose_th),
    path_x_(path_x),
    path_y_(path_y),
    dt_(dt),
    predictive_horizon_num_(predictive_horizon_num)
  {
  }

  template <typename T>
  bool operator()(T const * const * parameters, T * residual) const
  {
    std::vector<T> xs(predictive_horizon_num_ + 1, (T)init_pose_x_);
    std::vector<T> ys(predictive_horizon_num_ + 1, (T)init_pose_y_);
    std::vector<T> ths(predictive_horizon_num_ + 1, (T)init_pose_th_);

    for (int i = 0; i < predictive_horizon_num_; i++) {
      // clang-format off
      T x =
            xs[i] + 
              parameters[0][i] * cos(ths[i]) * dt_;
      T y = 
            ys[i] +
              parameters[0][i] * sin(ths[i]) * dt_;
      T th = 
            ths[i] + 
              parameters[1][i] * dt_;
      // clang-format on

      T cost = pow((path_x_[i] - x), 2) + pow((path_y_[i] - y), 2);

      xs[i + 1] = x;
      ys[i + 1] = y;
      ths[i + 1] = th;

      residual[i] = cost;
    }

    return true;
  }

private:
  double init_pose_x_, init_pose_y_, init_pose_th_;
  std::vector<double> path_x_;
  std::vector<double> path_y_;
  double dt_;
  int predictive_horizon_num_;
};

}  // namespace ike_nav

#endif  // IKE_CONTROLLER__IKE_CONTROLLER_HPP_
