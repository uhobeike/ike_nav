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
  double x, y, cost;

  Node() : x(0.0), y(0.0), cost(0.0) {}
  Node(double x, double y) : x(x), y(y), cost(0.0) {}
};

class IkePlanner : public rclcpp::Node
{
public:
  explicit IkePlanner(const rclcpp::NodeOptions & options);

protected:
  nav_msgs::msg::OccupancyGrid getMap();

  std::vector<ike_nav::Node> getObstacles(nav_msgs::msg::OccupancyGrid & map);
  std::vector<std::pair<double, double>> getObstaclesXY(std::vector<ike_nav::Node> obstacles_node);

  nav_msgs::msg::OccupancyGrid createGrid(nav_msgs::msg::OccupancyGrid & map);

  std::pair<double, double> calculateKey(ike_nav::Node node);

  double h(ike_nav::Node node);

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Service<ike_nav_msgs::srv::GetMap>::SharedPtr get_map_srv_;

  double x_min_world_, y_min_world_, x_max_, y_max_;
  std::vector<ike_nav::Node> obstacles_;
  std::vector<std::pair<double, double>> obstacles_xy_;
  ike_nav::Node start_, goal_;
  std::map<ike_nav::Node, std::pair<double, double>> U_;
  double km_, kold_;
  nav_msgs::msg::OccupancyGrid rhs_, g_;
  std::vector<std::pair<double, double>> detected_obstacles_xy_, xy_;
  bool initialized_;
};

}  // namespace ike_nav

#endif	// IKE_PLANNER__IKE_PLANNER_HPP_
