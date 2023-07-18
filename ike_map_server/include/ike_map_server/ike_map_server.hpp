// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef IKE_MAP_SERVER__IKE_MAP_SERVER_HPP_
#define IKE_MAP_SERVER__IKE_MAP_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"

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
  void initPublisher();
  void setParam();
  bool readMapYaml(Pgm & pgm);
  bool readPgm(Pgm & pgm);

private:
};

}  // namespace ike_nav

#endif	// IKE_MAP_SERVER__IKE_MAP_SERVER_HPP_
