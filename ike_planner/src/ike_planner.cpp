// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "ike_planner/ike_planner.hpp"

namespace ike_nav
{

IkePlanner::IkePlanner(const rclcpp::NodeOptions & options) : Node("ike_planner", options)
{
  auto map = nav_msgs::msg::OccupancyGrid();
  map = getMap();

  double lower_left, upper_left, lower_right, upper_right;
  lower_left = map.info.height;
  upper_right = map.info.width;

  // four corners of the map coordinate system
  x_min_world_ = 0.0;
  y_min_world_ = 0.0;
  x_max_ = x_min_world_ + map.info.width;
  y_max_ = y_min_world_ + map.info.height;

  obstacles_ = getObstacles(map);
  obstacles_xy_ = getObstaclesXY(obstacles_);

  start_ = ike_nav::Node(0.0, 0.0);
  goal_ = ike_nav::Node(0.0, 0.0);
  U_ = std::map<Node, double>();
  km_ = 0.0;
  kold_ = 0.0;
  rhs_ = nav_msgs::msg::OccupancyGrid();
  g_ = nav_msgs::msg::OccupancyGrid();
  detected_obstacles_xy_ = std::vector<std::pair<double, double>>();
  xy_ = std::vector<std::pair<double, double>>();
  initialized_ = false;
}

std::vector<ike_nav::Node> IkePlanner::getObstacles(nav_msgs::msg::OccupancyGrid & map)
{
  std::vector<ike_nav::Node> obstacles_node;
  for (unsigned int y = 0; y < map.info.height; y++) {
    for (unsigned int x = 0; x < map.info.width; x++) {
      unsigned int i = x + (map.info.height - y - 1) * map.info.width;
      if (map.data[i] == 100) obstacles_node.push_back(ike_nav::Node(x, y));
    }
  }

  return obstacles_node;
}

std::vector<std::pair<double, double>> IkePlanner::getObstaclesXY(
  std::vector<ike_nav::Node> obstacles_node)
{
  std::vector<std::pair<double, double>> obstacles_xy;

  for (auto node : obstacles_node)
    obstacles_xy.push_back(std::pair<double, double>(node.x, node.y));

  return obstacles_xy;
}

nav_msgs::msg::OccupancyGrid IkePlanner::getMap()
{
  auto get_map = this->create_client<ike_nav_msgs::srv::GetMap>("get_map");
  while (!get_map->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
  }
  auto request = std::make_shared<ike_nav_msgs::srv::GetMap::Request>();
  auto result_future = get_map->async_send_request(request);
  if (
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "service call failed :(");
    get_map->remove_pending_request(result_future);
  }

  return result_future.get()->map;

  // map_pub_ =
  //   this->create_publisher<nav_msgs::msg::OccupancyGrid>("get_map", rclcpp::QoS(1).reliable());

  // map_pub_->publish(result->map);
}

}  // namespace ike_nav

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ike_nav::IkePlanner)
