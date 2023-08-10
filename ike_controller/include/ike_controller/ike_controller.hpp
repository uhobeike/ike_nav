// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef IKE_CONTROLLER__IKE_CONTROLLER_HPP_
#define IKE_CONTROLLER__IKE_CONTROLLER_HPP_

#include <Eigen/Dense>
#include <nav2_util/robot_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ike_nav_msgs/srv/get_path.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <ceres/ceres.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using ceres::Solver;
namespace ike_nav
{
class IkeController : public rclcpp::Node
{
public:
  explicit IkeController(const rclcpp::NodeOptions & options);

protected:
  void initTf();
  void initServiceClient();
  void initLoopTimer();

  void asyncGetPath(geometry_msgs::msg::PoseStamped start, geometry_msgs::msg::PoseStamped goal);

  void getMapFrameRobotPose(geometry_msgs::msg::PoseStamped & map_frame_robot_pose);

  void ModelPredictiveControl();

  void loop();

private:
  rclcpp::Client<ike_nav_msgs::srv::GetPath>::SharedPtr get_path_client_;

  rclcpp::TimerBase::SharedPtr loop_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::msg::PoseStamped start_, goal_;
  nav_msgs::msg::Path path_;
};

struct ObjectiveFunction
{
  ObjectiveFunction(
    std::vector<double> path_x, std::vector<double> path_y, double dt, int predictive_horizon_num)
  : path_x_(path_x), path_y_(path_y), dt_(dt), predictive_horizon_num_(predictive_horizon_num)
  {
  }

  template <typename T>
  bool operator()(T const * const * parameters, T * residual) const
  {
    std::vector<T> xs(predictive_horizon_num_ + 1, (T)8.0);
    std::vector<T> ys(predictive_horizon_num_ + 1, (T)9.5);
    std::vector<T> ths(predictive_horizon_num_ + 1, (T)0.0);

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
  std::vector<double> path_x_;
  std::vector<double> path_y_;
  double dt_;
  int predictive_horizon_num_;
};

}  // namespace ike_nav

#endif  // IKE_CONTROLLER__IKE_CONTROLLER_HPP_
