// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "ike_map_server/ike_map_server.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <utility>

using namespace std::chrono_literals;

namespace ike_nav
{

IkeMapServer::IkeMapServer(const rclcpp::NodeOptions & options)
: Node("ike_map_server_composition", options)
{
  setParam();
  getParam();
  initPublisher();
}

void IkeMapServer::setParam() {}

void IkeMapServer::getParam() {}

void IkeMapServer::initPublisher() {}

}  // namespace ike_nav

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ike_nav::IkeMapServer)
