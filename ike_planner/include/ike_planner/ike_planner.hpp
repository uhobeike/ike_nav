// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef IKE_IKE_PLANNER__IKE_PLANNER_HPP_
#define IKE_IKE_PLANNER__IKE_IKE_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "ike_nav_msgs/srv/get_map.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

namespace ike_nav
{

struct Node
{
  uint32_t x, y, parent_index;
  double cost;

  Node() : x(0), y(0), cost(0.0) {}
  Node(uint32_t x, uint32_t y) : x(x), y(y), cost(0.0) {}
  Node(uint32_t x, uint32_t y, double cost, uint32_t parent_index)
  : x(x), y(y), cost(cost), parent_index(parent_index)
  {
  }
};

class IkePlanner : public rclcpp::Node
{
public:
  explicit IkePlanner(const rclcpp::NodeOptions & options);

protected:
  void initPublisher();

  void initializePlanner();

  nav_msgs::msg::OccupancyGrid getMap();

  std::vector<std::tuple<int32_t, int32_t, uint8_t>> getMotionModel();

  void planning(double sx, double sy, double gx, double gy);
  uint32_t calcXYIndex(double positio);
  uint32_t calcGridIndex(ike_nav::Node node);
  double calcHeurisic(ike_nav::Node node1, ike_nav::Node node2);
  bool verifyNode(ike_nav::Node node);
  void calcFinalPath(ike_nav::Node goal_node, std::map<uint32_t, ike_nav::Node> closed_set);
  double calcGridPosition(uint32_t goal_node_position);

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr search_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_path_pub_;
  rclcpp::Service<ike_nav_msgs::srv::GetMap>::SharedPtr get_map_srv_;

  double resolution_, robot_radius_;
  uint32_t min_x_, min_y_, max_x_, max_y_;
  nav_msgs::msg::OccupancyGrid * obstacle_map_;
  nav_msgs::msg::OccupancyGrid search_map_;
  uint32_t x_width_, y_width_;
  std::vector<std::tuple<int32_t, int32_t, uint8_t>> motion_;

  double start_x_, start_y_, goal_x_, goal_y_;
};

}  // namespace ike_nav

#endif  // IKE_PLANNER__IKE_PLANNER_HPP_
