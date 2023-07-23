// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "ike_planner/ike_planner.hpp"

namespace ike_nav
{

IkePlanner::IkePlanner(const rclcpp::NodeOptions & options) : Node("ike_planner", options)
{
	auto map = nav_msgs::msg::OccupancyGrid();
	map = getMap();

	resolution_ = map.info.resolution;
	robot_radius_ = 1.0;
	min_x_ = min_y_ = max_x_ = max_y_ = 0.0;
	*obstacle_map_ = nav_msgs::msg::OccupancyGrid();
	x_width_ = map.info.width;
	y_width_ = map.info.height;
	motion_ = getMotionModel();
	obstacle_map_ = &map;

	RCLCPP_INFO(this->get_logger(), "IkePlanner constructor done");
}

std::vector<std::tuple<double, double, uint8_t>> IkePlanner::getMotionModel()
{
	//dx, dy, cost
	return std::vector<std::tuple<double, double, uint8_t>>{
		{1, 0, 1},
		{0, 1, 1},
		{-1, 0, 1},
		{0, -1, 1},
		{-1, -1, std::sqrt(2)},
		{-1, 1, std::sqrt(2)},
		{1, -1, std::sqrt(2)},
		{1, 1, std::sqrt(2)}};
}

void IkePlanner::planning(double sx, double sy, double gx, double gy)
{
	auto start_node = ike_nav::Node(calcXYIndex(sx), calcXYIndex(sy), 0.0, -1);
	auto goal_node = ike_nav::Node(calcXYIndex(sx), calcXYIndex(sy), 0.0, -1);

	std::map<double, ike_nav::Node> open_set, closed_set;

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
            + calcHeurisic(goal_node, open_set[id_node_map.first])));
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

			if (closed_set.find(n_id) != closed_set.end()) continue;
			if (open_set.find(n_id) == open_set.end())
				open_set.insert(std::make_pair(n_id, node));
			else {
				if (open_set[n_id].cost > node.cost) open_set.insert(std::make_pair(n_id, node));
			}
		}
	}
}

bool IkePlanner::verifyNode(ike_nav::Node node) { return true; }

double IkePlanner::calcHeurisic(ike_nav::Node node1, ike_nav::Node node2)
{
	auto w = 1.0;
	auto d = w * std::hypot(node1.x - node2.x, node1.y - node2.y);

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

	// map_pub_ =
	//   this->create_publisher<nav_msgs::msg::OccupancyGrid>("get_map", rclcpp::QoS(1).reliable());

	// map_pub_->publish(result->map);
}

}	 // namespace ike_nav

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ike_nav::IkePlanner)
