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
  uint32_t x, y, cost, parent_index;

  Node() : x(0), y(0), cost(0.0) {}
  Node(uint32_t x, uint32_t y) : x(x), y(y), cost(0.0) {}
  Node(uint32_t x, uint32_t y, double cost, double parent_index)
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

  void planning(double sx, double sy, double gx, double gy);
  uint32_t calcXYIndex(double positio);
  uint32_t calcGridIndex(ike_nav::Node node);
  double calcHeurisic(ike_nav::Node node1, ike_nav::Node node2);
  bool verifyNode(ike_nav::Node node);
  std::pair<std::vector<double>, std::vector<double>> calcFinalPath(
    ike_nav::Node goal_node, std::map<double, ike_nav::Node> closed_set);
  double calcGridPosition(uint32_t goal_node_position);

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Service<ike_nav_msgs::srv::GetMap>::SharedPtr get_map_srv_;

  double resolution_, robot_radius_;
  double min_x_, min_y_, max_x_, max_y_;
  nav_msgs::msg::OccupancyGrid * obstacle_map_;
  uint32_t x_width_, y_width_;
  std::vector<std::tuple<double, double, uint8_t>> motion_;

  double start_x_, start_y_, goal_x_, goal_y_;
};

}  // namespace ike_nav

#endif  // IKE_PLANNER__IKE_PLANNER_HPP_
