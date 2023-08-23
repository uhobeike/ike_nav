// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef IKE_MAP_SERVER__IKE_MAP_SERVER_HPP_
#define IKE_MAP_SERVER__IKE_MAP_SERVER_HPP_

#include "ike_map_server_parameter/ike_map_server_parameter.hpp"

#include <rclcpp/rclcpp.hpp>

#include "ike_nav_msgs/srv/get_map.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace ike_nav
{

struct Pgm
{
  std::string header, image;
  int rows, cols, max_val, negate;
  double resolution, occupied_thresh, free_thresh;
  std::vector<unsigned char> pixels;
  std::vector<double> origin;
};

class IkeMapServer : public rclcpp::Node
{
public:
  explicit IkeMapServer(const rclcpp::NodeOptions & options);

protected:
  void getParam();

  void initPublisher();
  void initService();

  bool readMapYaml(Pgm & pgm);
  bool readPgm(Pgm & pgm);
  void createOccupancyGrid(Pgm & pgm, nav_msgs::msg::OccupancyGrid & occupancy_grid);
  void publishMap(nav_msgs::msg::OccupancyGrid & occupancy_grid);

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Service<ike_nav_msgs::srv::GetMap>::SharedPtr get_map_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr publish_map_srv_;

  std::shared_ptr<ike_map_server::ParamListener> param_listener_;
  ike_map_server::Params params_;

  std::string map_yaml_path_;
};

}  // namespace ike_nav

#endif  // IKE_MAP_SERVER__IKE_MAP_SERVER_HPP_
