// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef IKE_IKE_PLANNER__IKE_PLANNER_HPP_
#define IKE_IKE_PLANNER__IKE_IKE_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "ike_nav_msgs/srv/get_map.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace ike_nav
{

class IkePlanner : public rclcpp::Node
{
public:
  explicit IkePlanner(const rclcpp::NodeOptions & options);

protected:
private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Service<ike_nav_msgs::srv::GetMap>::SharedPtr get_map_srv_;
};

}  // namespace ike_nav

#endif	// IKE_PLANNER__IKE_PLANNER_HPP_
