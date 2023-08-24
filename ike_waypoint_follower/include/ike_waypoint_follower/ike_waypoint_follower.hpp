// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef IKE_WAYPOINT_FOLLOWER__IKE_WAYPOINT_FOLLOWER_HPP_
#define IKE_WAYPOINT_FOLLOWER__IKE_WAYPOINT_FOLLOWER_HPP_

#include <rclcpp/rclcpp.hpp>

namespace ike_nav
{
class IkeWaypointFollower : public rclcpp::Node
{
public:
  explicit IkeWaypointFollower(const rclcpp::NodeOptions & options);

protected:
private:
};

}  // namespace ike_nav

#endif  // IKE_WAYPOINT_FOLLOWER__IKE_WAYPOINT_FOLLOWER_HPP_
