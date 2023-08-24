// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef IKE_WAYPOINT_FOLLOWER__IKE_WAYPOINT_FOLLOWER_HPP_
#define IKE_WAYPOINT_FOLLOWER__IKE_WAYPOINT_FOLLOWER_HPP_

#include "ike_waypoint_follower_parameter/ike_waypoint_follower_parameter.hpp"

#include <rclcpp/rclcpp.hpp>

#include "ike_nav_msgs/msg/waypoints.hpp"

namespace ike_nav
{
class IkeWaypointFollower : public rclcpp::Node
{
public:
  explicit IkeWaypointFollower(const rclcpp::NodeOptions & options);

protected:
  void getParam();

  void readWaypointYaml();

private:
  std::shared_ptr<ike_waypoint_follower::ParamListener> param_listener_;
  ike_waypoint_follower::Params params_;

  std::string waypoint_yaml_path_;
  ike_nav_msgs::msg::Waypoints waypoints_;
};

}  // namespace ike_nav

#endif  // IKE_WAYPOINT_FOLLOWER__IKE_WAYPOINT_FOLLOWER_HPP_
