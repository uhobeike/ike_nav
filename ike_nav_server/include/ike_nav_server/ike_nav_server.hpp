// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef IKE_NAV_SERVER__IKE_NAV_SERVER_HPP_
#define IKE_NAV_SERVER__IKE_NAV_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "ike_nav_msgs/srv/get_path.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace ike_nav
{

class IkeNavServer : public rclcpp::Node
{
public:
  explicit IkeNavServer(const rclcpp::NodeOptions & options);

protected:
  void initServiceClient();
  void initLoopTimer();

  void asyncGetPath(geometry_msgs::msg::PoseStamped start, geometry_msgs::msg::PoseStamped goal);

  void loop();

private:
  rclcpp::Client<ike_nav_msgs::srv::GetPath>::SharedPtr get_path_client_;

  rclcpp::TimerBase::SharedPtr loop_timer_;

  geometry_msgs::msg::PoseStamped start_, goal_;
};

}  // namespace ike_nav

#endif  // IKE_MAP_SERVER__IKE_MAP_SERVER_HPP_
