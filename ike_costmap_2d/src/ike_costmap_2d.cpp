// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "ike_costmap_2d/ike_costmap_2d.hpp"

#include <algorithm>

namespace ike_nav
{

IkeCostMap2D::IkeCostMap2D(const rclcpp::NodeOptions & options) : Node("ike_costmap_2d", options)
{
  initPublisher();
  initService();
  map_ = getMap();
}

void IkeCostMap2D::initPublisher()
{
  static_layer_pub_ =
    this->create_publisher<nav_msgs::msg::OccupancyGrid>("static_layer", rclcpp::QoS(1).reliable());
  inflation_layer_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "inflation_layer", rclcpp::QoS(1).reliable());
  obstacle_layer_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "obstacle_layer", rclcpp::QoS(1).reliable());
}

void IkeCostMap2D::initService()
{
  auto get_costmap_2d =
    [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      [[maybe_unused]] const std::shared_ptr<ike_nav_msgs::srv::GetCostMap2D_Request> request,
      std::shared_ptr<ike_nav_msgs::srv::GetCostMap2D_Response> response) -> void {
    (void)request_header;

    auto costmap_2d_layers = createCostMap2DLayers(map_);

    response->costmap_2d = costmap_2d_layers["inflation_layer"];
    response->success = true;
    response->message = "Called /get_costmap_2d. Send map done.";
  };
  get_costmap_2d_srv_ =
    create_service<ike_nav_msgs::srv::GetCostMap2D>("get_costmap_2d", get_costmap_2d);

  auto publish_costmap_2d =
    [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger_Request> request,
      std::shared_ptr<std_srvs::srv::Trigger_Response> response) -> void {
    (void)request_header;

    auto costmap_2d_layers = createCostMap2DLayers(map_);

    publishCostMap2DLayers(costmap_2d_layers);

    response->success = true;
    response->message = "Called /publish_costmap_2d. Send map done.";
  };
  publish_map_srv_ =
    create_service<std_srvs::srv::Trigger>("publish_costmap_2d", publish_costmap_2d);
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
  auto inflation_radius = 10.0;

  for (uint32_t map_y = 0; map_y < inflation_layer.info.width; map_y++) {
    for (uint32_t map_x = 0; map_x < inflation_layer.info.height; map_x++)
      if (
        inflation_layer
          .data[inflation_layer.info.width * (inflation_layer.info.height - map_y - 1) + map_x] ==
        100) {
        calculateInflation(inflation_layer, inflation_radius, map_x, map_y);
      }
  }

  return inflation_layer;
};

void IkeCostMap2D::calculateInflation(
  nav_msgs::msg::OccupancyGrid & map, const double & inflation_radius, const uint32_t & map_x,
  const uint32_t & map_y)
{
  auto sub_map_x_start = map_x - inflation_radius;
  auto sub_map_y_start = map_y - inflation_radius;
  auto sub_map_x_end = map_x + inflation_radius;
  auto sub_map_y_end = map_y + inflation_radius;

  for (auto y = sub_map_y_start - inflation_radius; y < sub_map_y_end; y++)
    for (auto x = sub_map_x_start - inflation_radius; x < sub_map_x_end; x++)
      if (hypot(x - map_x, y - map_y) < inflation_radius)
        if (
          normalizeCost(
            calculateCost(0., inflation_radius),
            calculateCost(hypot(x - map_x, y - map_y), inflation_radius)) >
          map.data[map.info.width * (map.info.height - y - 1) + x])
          map.data[map.info.width * (map.info.height - y - 1) + x] = normalizeCost(
            calculateCost(0., inflation_radius),
            calculateCost(hypot(x - map_x, y - map_y), inflation_radius));
}

double IkeCostMap2D::calculateCost(double stochastic_variable, double inflation_radius)
{
  double sigma = inflation_radius / 3.;
  double cost = 1. / std::sqrt(2. * M_PI * sigma * sigma) *
                std::exp(-stochastic_variable * stochastic_variable / (2. * sigma * sigma));
  return cost;
}

double IkeCostMap2D::normalizeCost(double max_pdf, double pdf) { return (pdf / max_pdf) * 100.; }

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
