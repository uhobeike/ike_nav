// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "ike_waypoint_follower/ike_waypoint_follower.hpp"

namespace ike_nav
{

IkeWaypointFollower::IkeWaypointFollower(const rclcpp::NodeOptions & options)
: Node("ike_waypoint_follower", options)
{
}

}  // namespace ike_nav

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ike_nav::IkeWaypointFollower)
