// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef IKE_MAP_SERVER__IKE_MAP_SERVER_HPP_
#define IKE_MAP_SERVER__IKE_MAP_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"

namespace ike_nav
{

class IkeMapServer : public rclcpp::Node
{
public:
  explicit IkeMapServer(const rclcpp::NodeOptions & options);

protected:
  void initPublisher();
  void setParam();
  void getParam();

private:
  std::string map_yaml_path_;
};

}  // namespace ike_nav

#endif	// IKE_MAP_SERVER__IKE_MAP_SERVER_HPP_
