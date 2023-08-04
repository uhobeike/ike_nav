// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef IKE_NAV_SERVER__IKE_NAV_SERVER_HPP_
#define IKE_NAV_SERVER__IKE_NAV_SERVER_HPP_

#include <nav2_util/robot_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ike_nav_msgs/srv/get_path.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ike_nav
{

class IkeNavServer : public rclcpp::Node
{
public:
  explicit IkeNavServer(const rclcpp::NodeOptions & options);

protected:
  void initTf();
  void initServiceClient();
  void initLoopTimer();

  void asyncGetPath(geometry_msgs::msg::PoseStamped start, geometry_msgs::msg::PoseStamped goal);

  void getMapFrameRobotPose(geometry_msgs::msg::PoseStamped & map_frame_robot_pose);

  void loop();

private:
  rclcpp::Client<ike_nav_msgs::srv::GetPath>::SharedPtr get_path_client_;

  rclcpp::TimerBase::SharedPtr loop_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::msg::PoseStamped start_, goal_;
};

}  // namespace ike_nav

#endif  // IKE_NAV_SERVER__IKE_NAV_SERVER_HPP_
