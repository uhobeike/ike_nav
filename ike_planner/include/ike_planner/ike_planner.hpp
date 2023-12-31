// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef IKE_PLANNER__IKE_PLANNER_HPP_
#define IKE_PLANNER__IKE_PLANNER_HPP_

#include "ike_planner_parameter/ike_planner_parameter.hpp"

#include <rclcpp/rclcpp.hpp>

#include "ike_nav_msgs/srv/get_cost_map2_d.hpp"
#include "ike_nav_msgs/srv/get_path.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

namespace ike_nav
{
struct Node
{
  uint32_t x, y;
  double cost;
  int32_t parent_index;

  Node() : x(0), y(0), cost(0.0) {}
  Node(uint32_t x, uint32_t y) : x(x), y(y), cost(0.0) {}
  Node(uint32_t x, uint32_t y, double cost, int32_t parent_index)
  : x(x), y(y), cost(cost), parent_index(parent_index)
  {
  }
};

class IkePlanner : public rclcpp::Node
{
public:
  explicit IkePlanner(const rclcpp::NodeOptions & options);

protected:
  void getParam();

  void initPublisher();
  void initSubscriber();
  void initServiceServer();
  void initServiceClient();

  void initPlanner();

  void getCostMap2D();

  std::vector<std::tuple<int32_t, int32_t, uint8_t>> getMotionModel();

  nav_msgs::msg::Path planning(double sx, double sy, double gx, double gy);
  uint32_t calcXYIndex(double positio);
  uint32_t calcGridIndex(ike_nav::Node node);
  double calcHeurisic(ike_nav::Node node1, ike_nav::Node node2);
  bool verifyNode(ike_nav::Node node);
  nav_msgs::msg::Path calcFinalPath(
    ike_nav::Node goal_node, std::map<uint32_t, ike_nav::Node> closed_set);
  double calcGridPosition(uint32_t goal_node_position);

  void smoothPath(nav_msgs::msg::Path & path);
  nav_msgs::msg::Path smoothOptimization(nav_msgs::msg::Path & path);
  double calcNewPositionXY(
    double & delta, double original_data, double smoothed_data, double smoothed_prev_data,
    double smoothed_next_data);

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr search_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_path_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_2d_sub_;
  rclcpp::Service<ike_nav_msgs::srv::GetPath>::SharedPtr get_path_srv_;
  rclcpp::Client<ike_nav_msgs::srv::GetCostMap2D>::SharedPtr get_costmap_2d_map_srv_client_;

  std::shared_ptr<ike_planner::ParamListener> param_listener_;
  ike_planner::Params params_;

  double resolution_, robot_radius_;
  uint32_t min_x_, min_y_, max_x_, max_y_;
  nav_msgs::msg::OccupancyGrid obstacle_map_;
  nav_msgs::msg::OccupancyGrid search_map_;
  uint32_t x_width_, y_width_;
  std::vector<std::tuple<int32_t, int32_t, uint8_t>> motion_;

  bool use_dijkstra_, publish_searched_map_;
  double update_path_weight_, smooth_path_weight_, iteration_delta_threshold_;

  // todo
  double max_smooth_path_iteration_;
};

}  // namespace ike_nav

#endif  // IKE_PLANNER__IKE_PLANNER_HPP_
