// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "ike_planner/ike_planner.hpp"

namespace ike_nav
{

IkePlanner::IkePlanner(const rclcpp::NodeOptions & options) : Node("ike_planner", options)
{
  auto map = nav_msgs::msg::OccupancyGrid();
  map = getMap();

  // double lower_left, upper_left, lower_right, upper_right;
  // lower_left = map.info.height;
  // upper_right = map.info.width;

  // four corners of the map coordinate system
  x_min_world_ = 0.0;
  y_min_world_ = 0.0;
  x_max_ = x_min_world_ + map.info.width;
  y_max_ = y_min_world_ + map.info.height;

  obstacles_ = getObstacles(map);
  obstacles_xy_ = getObstaclesXY(obstacles_);

  start_ = ike_nav::Node(0.0, 0.0);
  goal_ = ike_nav::Node(0.0, 0.0);
  U_.insert(std::make_pair(goal_, calculateKey(goal_)));
  km_ = 0.0;
  kold_ = std::pair<double, double>();
  rhs_ = createGrid(map);
  g_ = createGrid(map);
  detected_obstacles_xy_ = std::vector<std::pair<double, double>>();
  xy_ = std::vector<std::pair<double, double>>();
  initialized_ = true;
  RCLCPP_INFO(get_logger(), "IkePlanner initialized done");

  computeShortestPath();
}

void IkePlanner::computeShortestPath()
{
  bool has_elements = false;
  has_elements = U_.size() > 0;

  bool rhs_not_equal_to_g = false;
  rhs_not_equal_to_g = rhs_.data[start_.x + start_.y] != g_.data[start_.x + start_.y];

  while (true) {
    kold_ = U_.begin()->second;
    ike_nav::Node u = U_.begin()->first;
    U_.erase(U_.begin());
    if (true) {
    } else if (true) {
    } else {
    }
  }
}

std::pair<double, double> IkePlanner::calculateKey(ike_nav::Node s)
{
  return std::pair<double, double>(
    std::min(g_.data[s.x + s.y], rhs_.data[s.x + s.y]) + h(s) + km_,
    std::min(g_.data[s.x + s.y], rhs_.data[s.x + s.y]));
}

bool compareKeys(std::pair<double, double> key_pair1, std::pair<double, double> key_pair2)
{
  return key_pair1.first < key_pair2.first ||
	 (key_pair1.first == key_pair2.first and key_pair1.second < key_pair2.second);
}

double IkePlanner::h(ike_nav::Node node) { return 1; }

std::vector<ike_nav::Node> IkePlanner::getObstacles(nav_msgs::msg::OccupancyGrid & map)
{
  auto obstacles_node = std::vector<ike_nav::Node>();
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
  auto obstacles_xy = std::vector<std::pair<double, double>>();
  for (auto node : obstacles_node)
    obstacles_xy.push_back(std::pair<double, double>(node.x, node.y));

  return obstacles_xy;
}

nav_msgs::msg::OccupancyGrid IkePlanner::createGrid(nav_msgs::msg::OccupancyGrid & map)
{
  nav_msgs::msg::OccupancyGrid grid = map;
  constexpr int8_t inf = 127;
  for (auto & data : grid.data) data = inf;

  return grid;
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
