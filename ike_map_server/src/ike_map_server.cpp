// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "ike_map_server/ike_map_server.hpp"

#include <rclcpp/rclcpp.hpp>

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
  Pgm pgm;

  initPublisher();
  setParam();
  readMapYaml(pgm);

  readPgm(pgm);
  RCLCPP_INFO(get_logger(), "%s", this->get_parameter("map_yaml_path").as_string().c_str());
}
void IkeMapServer::initPublisher() {}

void IkeMapServer::setParam() { this->declare_parameter("map_yaml_path", ""); }

bool IkeMapServer::readMapYaml(Pgm & pgm)
{
  YAML::Node config = YAML::LoadFile(this->get_parameter("map_yaml_path").as_string().c_str());

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
  if (found != std::string::npos)
    pgm_path.erase(found);
  else
    return false;

  std::ifstream file(pgm_path, std::ios::binary);
  if (!file) {
    std::cerr << "Cannot open file!\n";
    return false;
  }
  file >> pgm.header;
  if (pgm.header != "P5") {  // assuming the image is in P5 format
    std::cerr << "Can only handle P5 format!\n";
    return false;
  }
  file >> pgm.cols >> pgm.rows >> pgm.max_val;
  if (pgm.max_val > 255) {
    std::cerr << "Can only handle 8-bit pixels!\n";
    return false;
  }
  file.ignore(1);  // skip newline character

  pgm.pixels.resize(pgm.rows * pgm.cols);
  file.read(reinterpret_cast<char *>(pgm.pixels.data()), pgm.pixels.size());

  return true;
}

}  // namespace ike_nav

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ike_nav::IkeMapServer)
