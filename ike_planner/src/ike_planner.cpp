// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "ike_planner/ike_planner.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace ike_nav
{

IkePlanner::IkePlanner(const rclcpp::NodeOptions & options) : Node("ike_planner", options)
{
  declareParam();
  getParam();

  search_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "planner_searched_map", rclcpp::QoS(1).reliable());
  plan_path_pub_ =
    this->create_publisher<nav_msgs::msg::Path>("plan_path", rclcpp::QoS(1).reliable());

  auto map = nav_msgs::msg::OccupancyGrid();
  map = getMap();

  resolution_ = map.info.resolution;
  robot_radius_ = 1.0;
  min_x_ = min_y_ = 0;
  max_x_ = x_width_ = map.info.width;
  max_y_ = y_width_ = map.info.height;
  motion_ = getMotionModel();
  obstacle_map_ = &map;
  search_map_ = map;

  start_x_ = 8.1;
  start_y_ = 11.1;
  goal_x_ = 11.6;
  goal_y_ = 8.7;

  RCLCPP_INFO(this->get_logger(), "IkePlanner planning start");
  planning(start_x_, start_y_, goal_x_, goal_y_);
  RCLCPP_INFO(this->get_logger(), "IkePlanner planning done");
}

void IkePlanner::declareParam()
{
  declare_parameter("use_dijkstra", false);
  declare_parameter("publish_searched_map", true);
}
void IkePlanner::getParam()
{
  get_parameter("use_dijkstra", use_dijkstra_);
  get_parameter("publish_searched_map", publish_searched_map_);
}

std::vector<std::tuple<int32_t, int32_t, uint8_t>> IkePlanner::getMotionModel()
{
  // dx, dy, cost
  return std::vector<std::tuple<int32_t, int32_t, uint8_t>>{
    {1, 0, 1},
    {0, 1, 1},
    {-1, 0, 1},
    {0, -1, 1},
    {-1, -1, std::sqrt(2)},
    {-1, 1, std::sqrt(2)},
    {1, -1, std::sqrt(2)},
    {1, 1, std::sqrt(2)}};
}

std::pair<std::vector<double>, std::vector<double>> IkePlanner::planning(
  double sx, double sy, double gx, double gy)
{
  auto start_node = ike_nav::Node(calcXYIndex(sx), calcXYIndex(sy), 0.0, -1);
  auto goal_node = ike_nav::Node(calcXYIndex(gx), calcXYIndex(gy), 0.0, -1);

  std::map<uint32_t, ike_nav::Node> open_set, closed_set;

  open_set.insert(std::make_pair(calcGridIndex(start_node), start_node));

  while (rclcpp::ok()) {
    if (open_set.size() == 0) {
      RCLCPP_ERROR(this->get_logger(), "Open set is empty");
      rclcpp::shutdown();
    }

    // clang-format off
    auto c_id = [&]() -> uint32_t {
      std::map<uint32_t, double> id_cost_map;
      for (auto id_node_map : open_set) {
	      id_cost_map.insert(std::make_pair(
	        id_node_map.first, open_set[id_node_map.first].cost 
            + calcHeurisic(goal_node, static_cast<ike_nav::Node>(open_set[id_node_map.first]))));
      }

      return std::min_element(
	       id_cost_map.begin(), id_cost_map.end(),
	       [](const auto & a, const auto & b) 
         { return a.second < b.second; }
         )->first;
    }();
    // clang-format on

    auto current = open_set[c_id];

    if (current.x == goal_node.x and current.y == goal_node.y) {
      RCLCPP_INFO(this->get_logger(), "Find goal");
      goal_node.parent_index = current.parent_index;
      goal_node.cost = current.cost;
      break;
    }

    open_set.erase(c_id);

    closed_set.insert(std::make_pair(c_id, current));

    for (size_t i = 0; i < motion_.size(); ++i) {
      auto node = ike_nav::Node(
        current.x + std::get<0>(motion_[i]), current.y + std::get<1>(motion_[i]),
        current.cost + std::get<2>(motion_[i]), c_id);

      auto n_id = calcGridIndex(node);

      if (!verifyNode(node)) continue;

      // check motion
      search_map_.data[n_id] = 50;

      if (closed_set.find(n_id) != closed_set.end()) continue;
      if (open_set.find(n_id) == open_set.end())
        open_set.insert(std::make_pair(n_id, node));
      else {
        if (open_set[n_id].cost > node.cost) open_set.insert(std::make_pair(n_id, node));
      }
    }
  }

  return calcFinalPath(goal_node, closed_set);
}

std::pair<std::vector<double>, std::vector<double>> IkePlanner::calcFinalPath(
  ike_nav::Node goal_node, std::map<uint32_t, ike_nav::Node> closed_set)
{
  std::vector<double> rx, ry;
  rx.push_back(calcGridPosition(goal_node.x));
  ry.push_back(calcGridPosition(goal_node.y));

  auto parent_index = goal_node.parent_index;

  // check calcFinalPath
  auto plan_path = nav_msgs::msg::Path();
  auto pose_stamp = geometry_msgs::msg::PoseStamped();

  while (parent_index != -1) {
    auto n = closed_set[parent_index];
    rx.push_back(calcGridPosition(n.x));
    ry.push_back(calcGridPosition(n.y));
    parent_index = n.parent_index;

    pose_stamp.pose.position.x = calcGridPosition(n.x);
    pose_stamp.pose.position.y = calcGridPosition(n.y);
    plan_path.poses.push_back(pose_stamp);
  }

  plan_path.header.frame_id = "map";
  plan_path.header.stamp = rclcpp::Time(0);

  if (publish_searched_map_) search_map_pub_->publish(search_map_);
  plan_path_pub_->publish(plan_path);

  return std::make_pair(rx, ry);
}

double IkePlanner::calcGridPosition(uint32_t node_position) { return node_position * resolution_; }

bool IkePlanner::verifyNode(ike_nav::Node node)
{
  if (node.x < min_x_)
    return false;
  else if (node.y < min_y_)
    return false;
  else if (node.x >= max_x_)
    return false;
  else if (node.y >= max_y_)
    return false;

  if (obstacle_map_->data[calcGridIndex(node)]) return false;

  return true;
}

double IkePlanner::calcHeurisic(ike_nav::Node node1, ike_nav::Node node2)
{
  auto w = 1.0;
  double d = w * std::hypot(
                   static_cast<double>(node1.x) - static_cast<double>(node2.x),
                   static_cast<double>(node1.y) - static_cast<double>(node2.y));

  // if Dijkstra's algorithm
  if (use_dijkstra_) d = 0.0;

  return d;
}

uint32_t IkePlanner::calcXYIndex(double position)
{
  return static_cast<uint32_t>(std::round(position / resolution_));
}

uint32_t IkePlanner::calcGridIndex(ike_nav::Node node) { return node.y * x_width_ + node.x; }

nav_msgs::msg::OccupancyGrid IkePlanner::getMap()
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

}  // namespace ike_nav

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ike_nav::IkePlanner)
