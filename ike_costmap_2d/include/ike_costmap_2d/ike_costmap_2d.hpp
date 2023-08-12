// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef IKE_COSTMAP_2D__IKE_COSTMAP_2D_HPP_
#define IKE_COSTMAP_2D__IKE_COSTMAP_2D_HPP_

#include <rclcpp/rclcpp.hpp>

#include "ike_nav_msgs/srv/get_cost_map2_d.hpp"
#include "ike_nav_msgs/srv/get_map.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace ike_nav
{

class IkeCostMap2D : public rclcpp::Node
{
public:
  explicit IkeCostMap2D(const rclcpp::NodeOptions & options);

protected:
  void initPublisher();
  void initService();

  std::map<std::string, nav_msgs::msg::OccupancyGrid> createCostMap2DLayers(
    const nav_msgs::msg::OccupancyGrid & map);
  nav_msgs::msg::OccupancyGrid createStaticLayer(const nav_msgs::msg::OccupancyGrid & map);
  nav_msgs::msg::OccupancyGrid createInflationLayer(const nav_msgs::msg::OccupancyGrid & map);

  nav_msgs::msg::OccupancyGrid getMap();

  void publishCostMap2DLayers(
    std::map<std::string, nav_msgs::msg::OccupancyGrid> & costmap_2d_layers);

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr static_layer_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflation_layer_pub_;
  // rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_layer_pub_;
  rclcpp::Service<ike_nav_msgs::srv::GetMap>::SharedPtr get_costmap_2d_srv_;
  rclcpp::Service<ike_nav_msgs::srv::GetMap>::SharedPtr get_map_srv_;

  std::map<std::string, nav_msgs::msg::OccupancyGrid> costmap_2d_layers_;
};

}  // namespace ike_nav

#endif  // IKE_COSTMAP_2D__IKE_COSTMAP_2D_HPP_
