// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef IKE_IKE_PLANNER__IKE_PLANNER_HPP_
#define IKE_IKE_PLANNER__IKE_IKE_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "ike_nav_msgs/srv/get_map.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace ike_nav
{

struct Node
{
  double x, y, cost, parent_index;

  Node() : x(0.0), y(0.0), cost(0.0) {}
  Node(double x, double y) : x(x), y(y), cost(0.0) {}
  Node(double x, double y, double cost, double parent_index)
  : x(x), y(y), cost(cost), parent_index(parent_index)
  {
  }
};

class IkePlanner : public rclcpp::Node
{
public:
  explicit IkePlanner(const rclcpp::NodeOptions & options);

protected:
  nav_msgs::msg::OccupancyGrid getMap();

  std::vector<std::tuple<double, double, uint8_t>> getMotionModel();

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Service<ike_nav_msgs::srv::GetMap>::SharedPtr get_map_srv_;

  double resolution_, robot_radius_;
  double min_x_, min_y_, max_x_, max_y_;
  nav_msgs::msg::OccupancyGrid * obstacle_map_;
  uint32_t x_width, y_width;
  std::vector<std::tuple<double, double, uint8_t>> motion_;
};

}  // namespace ike_nav

#endif	// IKE_PLANNER__IKE_PLANNER_HPP_
