// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "ike_map_server/ike_map_server.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace ike_nav
{

IkeMapServer::IkeMapServer(const rclcpp::NodeOptions & options) : Node("ike_map_server", options)
{
  getParam();

  initPublisher();
  initService();

  RCLCPP_INFO(
    get_logger(), "Read map yaml: %s", this->get_parameter("map_yaml_path").as_string().c_str());

  Pgm pgm;
  readMapYaml(pgm);
  readPgm(pgm);

  nav_msgs::msg::OccupancyGrid map;
  createOccupancyGrid(pgm, map);
  publishMap(map);
}

void IkeMapServer::getParam()
{
  this->param_listener_ =
    std::make_shared<ike_map_server::ParamListener>(this->get_node_parameters_interface());
  this->params_ = param_listener_->get_params();

  map_yaml_path_ = this->params_.map_yaml_path;
}

void IkeMapServer::initPublisher()
{
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).reliable());
}

void IkeMapServer::initService()
{
  auto get_map =
    [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      [[maybe_unused]] const std::shared_ptr<ike_nav_msgs::srv::GetMap_Request> request,
      std::shared_ptr<ike_nav_msgs::srv::GetMap_Response> response) -> void {
    (void)request_header;

    Pgm pgm;
    RCLCPP_INFO(
      get_logger(), "Read map yaml: %s", this->get_parameter("map_yaml_path").as_string().c_str());
    readMapYaml(pgm);
    readPgm(pgm);

    nav_msgs::msg::OccupancyGrid map;
    createOccupancyGrid(pgm, map);

    response->map = map;
    response->success = true;
    response->message = "Called /get_map. Send map done.";
  };
  get_map_srv_ = create_service<ike_nav_msgs::srv::GetMap>("get_map", get_map);

  auto publish_map =
    [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger_Request> request,
      std::shared_ptr<std_srvs::srv::Trigger_Response> response) -> void {
    (void)request_header;

    Pgm pgm;
    RCLCPP_INFO(
      get_logger(), "Read map yaml: %s", this->get_parameter("map_yaml_path").as_string().c_str());
    readMapYaml(pgm);
    readPgm(pgm);

    nav_msgs::msg::OccupancyGrid map;
    createOccupancyGrid(pgm, map);
    publishMap(map);

    response->success = true;
    response->message = "Called /publish_map. Publish map done.";
  };
  publish_map_srv_ = create_service<std_srvs::srv::Trigger>("publish_map", publish_map);
}

bool IkeMapServer::readMapYaml(Pgm & pgm)
{
  YAML::Node config = YAML::LoadFile(map_yaml_path_);

  pgm.image = config["image"].as<std::string>();
  pgm.resolution = config["resolution"].as<double>();
  pgm.origin = config["origin"].as<std::vector<double>>();
  pgm.negate = config["negate"].as<int>();
  pgm.occupied_thresh = config["occupied_thresh"].as<double>();
  pgm.free_thresh = config["free_thresh"].as<double>();
  return false;
}

bool IkeMapServer::readPgm(Pgm & pgm)
{
  std::string pgm_path = this->get_parameter("map_yaml_path").as_string();
  std::size_t found = pgm_path.find_last_of("/");
  if (found != std::string::npos) {
    pgm_path.erase(found);
    pgm_path = pgm_path + "/" + pgm.image;
  } else
    return false;

  std::ifstream file(pgm_path, std::ios::binary);
  if (!file) {
    std::cerr << "Cannot open file!\n";
    return false;
  } else
    RCLCPP_INFO(get_logger(), "Open pgm: %s", pgm_path.c_str());

  file >> pgm.header;
  if (pgm.header != "P5") {  // assuming the image is in P5 format
    std::cerr << "Can only handle P5 format!\n";
    return false;
  }

  std::string check_cols;
  file >> check_cols;
  if (check_cols == "#") {
    file >> check_cols >> check_cols >> check_cols >> check_cols;
    file >> pgm.cols >> pgm.rows >> pgm.max_val;
  } else {
    pgm.cols = std::stoi(check_cols);
    file >> pgm.rows >> pgm.max_val;
  }

  if (pgm.max_val > 255) {
    std::cerr << "Can only handle 8-bit pixels!\n";
    return false;
  }
  file.ignore(1);  // skip newline character

  RCLCPP_INFO(get_logger(), "Read pgm: %s", pgm_path.c_str());
  pgm.pixels.resize(pgm.rows * pgm.cols);
  file.read(reinterpret_cast<char *>(pgm.pixels.data()), pgm.pixels.size());
  RCLCPP_INFO(get_logger(), "Read pgm done: %s", pgm_path.c_str());

  return true;
}

void IkeMapServer::createOccupancyGrid(Pgm & pgm, nav_msgs::msg::OccupancyGrid & occupancy_grid)
{
  occupancy_grid.header.stamp = now();
  occupancy_grid.header.frame_id = "map";

  occupancy_grid.info.map_load_time = now();
  occupancy_grid.info.resolution = pgm.resolution;
  occupancy_grid.info.width = pgm.cols;
  occupancy_grid.info.height = pgm.rows;
  occupancy_grid.info.origin.position.x = 0;
  occupancy_grid.info.origin.position.y = 0;
  occupancy_grid.info.origin.orientation.w = cos(pgm.origin[2] / 2);
  occupancy_grid.info.origin.orientation.z = sin(pgm.origin[2] / 2);

  int map_size = occupancy_grid.info.width * occupancy_grid.info.height;
  occupancy_grid.data.resize(map_size);

  for (auto & pixel : pgm.pixels) {
    if (pixel >= 254)
      pixel = 0;
    else if (pixel == 205)
      pixel = -1;
    else if (pixel == 0)
      pixel = 100;
  }

  std::vector<int8_t> pixels;
  for (auto & pixel : pgm.pixels) {
    pixels.push_back(static_cast<int8_t>(pixel));
  }

  unsigned int index = 0;
  for (unsigned int y = 0; y < occupancy_grid.info.height; y++) {
    for (unsigned int x = 0; x < occupancy_grid.info.width; x++) {
      unsigned int i = x + (occupancy_grid.info.height - y - 1) * occupancy_grid.info.width;
      occupancy_grid.data[i] = pixels[index];
      ++index;
    }
  }
}

void IkeMapServer::publishMap(nav_msgs::msg::OccupancyGrid & occupancy_grid)
{
  map_pub_->publish(occupancy_grid);
  RCLCPP_INFO(get_logger(), "Publish occupancy grid done");
}

}  // namespace ike_nav

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ike_nav::IkeMapServer)
