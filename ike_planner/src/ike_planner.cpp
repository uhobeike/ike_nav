// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "ike_planner/ike_planner.hpp"

namespace ike_nav
{

IkePlanner::IkePlanner(const rclcpp::NodeOptions & options) : Node("ike_planner", options)
{
  auto map = nav_msgs::msg::OccupancyGrid();
  map = getMap();

  resolution_ = map.info.resolution;
  robot_radius_ = 1.0;
  min_x_ = min_y_ = max_x_ = max_y_ = 0.0;
  obstacle_map_ = nav_msgs::msg::OccupancyGrid();
  x_width = y_width = 0;
}

std::vector<std::tuple<double, double, uint8_t>> IkePlanner::getMotionModel()
{
  //dx, dy, cost
  return std::vector<std::tuple<double, double, uint8_t>>{
    {1, 0, 1},
    {0, 1, 1},
    {-1, 0, 1},
    {0, -1, 1},
    {-1, -1, std::sqrt(2)},
    {-1, 1, std::sqrt(2)},
    {1, -1, std::sqrt(2)},
    {1, 1, std::sqrt(2)}};
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
