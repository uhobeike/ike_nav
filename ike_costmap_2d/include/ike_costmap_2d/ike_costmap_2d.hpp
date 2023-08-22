// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef IKE_COSTMAP_2D__IKE_COSTMAP_2D_HPP_
#define IKE_COSTMAP_2D__IKE_COSTMAP_2D_HPP_

#include <rclcpp/rclcpp.hpp>

#include "ike_nav_msgs/srv/get_cost_map2_d.hpp"
#include "ike_nav_msgs/srv/get_map.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ike_nav
{

class IkeCostMap2D : public rclcpp::Node
{
public:
  explicit IkeCostMap2D(const rclcpp::NodeOptions & options);

protected:
  void initTf();
  void initPublisher();
  void initSubscription();
  void initServiceServer();
  void initTimer();

  std::map<std::string, nav_msgs::msg::OccupancyGrid> createCostMap2DLayers(
    const nav_msgs::msg::OccupancyGrid & map);
  nav_msgs::msg::OccupancyGrid createStaticLayer(const nav_msgs::msg::OccupancyGrid & map);
  nav_msgs::msg::OccupancyGrid createInflationLayer(const nav_msgs::msg::OccupancyGrid & map);
  void createObstacleLayer();

  void calculateInflation(
    nav_msgs::msg::OccupancyGrid & map, const double & inflation_radius, const uint32_t & map_x,
    const uint32_t & map_y);
  double calculateCost(double stochastic_variable, double inflation_radius);
  double normalizeCost(double max_cost, double cost);

  std::vector<std::pair<uint32_t, uint32_t>> calculateHitPoint(
    sensor_msgs::msg::LaserScan scan, geometry_msgs::msg::PoseStamped lidar_pose);
  geometry_msgs::msg::PoseStamped getMapFrameRobotPose();
  inline double getRadian(double degree) { return degree * M_PI / 180; }

  nav_msgs::msg::OccupancyGrid getMap();

  void publishCostMap2DLayers(
    std::map<std::string, nav_msgs::msg::OccupancyGrid> & costmap_2d_layers);

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr static_layer_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflation_layer_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_layer_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Service<ike_nav_msgs::srv::GetCostMap2D>::SharedPtr get_costmap_2d_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr publish_map_srv_;
  rclcpp::Service<ike_nav_msgs::srv::GetMap>::SharedPtr get_map_srv_;

  rclcpp::TimerBase::SharedPtr create_obstacle_layer_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::map<std::string, nav_msgs::msg::OccupancyGrid> costmap_2d_layers_;

  nav_msgs::msg::OccupancyGrid map_;
  sensor_msgs::msg::LaserScan scan_;

  bool get_map_, get_scan_, get_lidar_pose_;
};

}  // namespace ike_nav

#endif  // IKE_COSTMAP_2D__IKE_COSTMAP_2D_HPP_
