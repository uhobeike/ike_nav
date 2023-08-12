// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "ike_costmap_2d/ike_costmap_2d.hpp"

#include <algorithm>

namespace ike_nav
{

IkeCostMap2D::IkeCostMap2D(const rclcpp::NodeOptions & options) : Node("ike_costmap_2d", options)
{
  initPublisher();

  auto costmap_2d_layers = createCostMap2DLayers(getMap());

  publishCostMap2DLayers(costmap_2d_layers);
}

void IkeCostMap2D::initPublisher()
{
  static_layer_pub_ =
    this->create_publisher<nav_msgs::msg::OccupancyGrid>("static_layer", rclcpp::QoS(1).reliable());
  inflation_layer_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "inflation_layer", rclcpp::QoS(1).reliable());
  // obstacle_layer_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
  //   "obstacle_layer", rclcpp::QoS(1).reliable());
}

nav_msgs::msg::OccupancyGrid IkeCostMap2D::getMap()
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
}

std::map<std::string, nav_msgs::msg::OccupancyGrid> IkeCostMap2D::createCostMap2DLayers(
  const nav_msgs::msg::OccupancyGrid & map)
{
  std::map<std::string, nav_msgs::msg::OccupancyGrid> costmap_2d_layers;

  costmap_2d_layers["static_layer"] = createStaticLayer(map);
  costmap_2d_layers["inflation_layer"] = createInflationLayer(map);

  return costmap_2d_layers;
}

nav_msgs::msg::OccupancyGrid IkeCostMap2D::createStaticLayer(
  const nav_msgs::msg::OccupancyGrid & map)
{
  return map;
};

nav_msgs::msg::OccupancyGrid IkeCostMap2D::createInflationLayer(
  const nav_msgs::msg::OccupancyGrid & map)
{
  auto inflation_layer = map;
  auto inflation_radius = 8.0;

  for (uint32_t map_y = 0; map_y < inflation_layer.info.width; map_y++)
    for (uint32_t map_x = 0; map_x < inflation_layer.info.height; map_x++)
      if (
        inflation_layer
          .data[inflation_layer.info.width * (inflation_layer.info.height - map_y - 1) + map_x] ==
        100) {
        auto sub_map_x_start = map_x - inflation_radius;
        auto sub_map_y_start = map_y - inflation_radius;
        auto sub_map_x_end = map_x + inflation_radius;
        auto sub_map_y_end = map_y + inflation_radius;

        for (auto y = sub_map_y_start - inflation_radius; y < sub_map_y_end; y++)
          for (auto x = sub_map_x_start - inflation_radius; x < sub_map_x_end; x++)
            if (hypot(x - map_x, y - map_y) < inflation_radius) {
              inflation_layer
                .data[inflation_layer.info.width * (inflation_layer.info.height - y - 1) + x] = 99;
            }
      }

  for (auto & map_data : inflation_layer.data)
    if (map_data == 99) map_data = 100;

  return inflation_layer;
};

void IkeCostMap2D::publishCostMap2DLayers(
  std::map<std::string, nav_msgs::msg::OccupancyGrid> & costmap_2d_layers)
{
  costmap_2d_layers["static_layer"].header.stamp = rclcpp::Time();
  costmap_2d_layers["inflation_layer"].header.stamp = rclcpp::Time();

  static_layer_pub_->publish(costmap_2d_layers["static_layer"]);
  inflation_layer_pub_->publish(costmap_2d_layers["inflation_layer"]);
}

}  // namespace ike_nav

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ike_nav::IkeCostMap2D)
