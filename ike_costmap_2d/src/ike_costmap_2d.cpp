// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "ike_costmap_2d/ike_costmap_2d.hpp"

#include <nav2_util/robot_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>

namespace ike_nav
{

IkeCostMap2D::IkeCostMap2D(const rclcpp::NodeOptions & options) : Node("ike_costmap_2d", options)
{
  getParam();

  initTf();
  initPublisher();
  initSubscription();
  initServiceServer();
  initServiceClient();
  initTimer();

  getMap();
}

void IkeCostMap2D::getParam()
{
  this->param_listener_ =
    std::make_shared<ike_costmap_2d::ParamListener>(this->get_node_parameters_interface());
  this->params_ = param_listener_->get_params();

  inflation_layer_inflation_radius_ = this->params_.inflation_layer.inflation_radius;
  obstacle_layer_inflation_radius_ = this->params_.obstacle_layer.inflation_radius;
  obstacle_layer_obstacle_range_ = this->params_.obstacle_layer.obstacle_range;
  publish_costmap_2d_ms_ = 1 / this->params_.publish_costmap_2d_hz * 1000.;
}

void IkeCostMap2D::initTf()
{
  tf_buffer_.reset();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  // tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void IkeCostMap2D::initPublisher()
{
  static_layer_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "static_layer", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  inflation_layer_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "inflation_layer", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  obstacle_layer_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "obstacle_layer", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  rclcpp::PublisherOptions options;
  options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  costmap_2d_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "costmap_2d", rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().reliable(), options);
}

void IkeCostMap2D::initSubscription()
{
  auto scan_callback = [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
    scan_ = *msg;
    get_scan_ = true;
  };

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(), scan_callback);
}

void IkeCostMap2D::initServiceServer()
{
  auto get_costmap_2d =
    [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      [[maybe_unused]] const std::shared_ptr<ike_nav_msgs::srv::GetCostMap2D_Request> request,
      std::shared_ptr<ike_nav_msgs::srv::GetCostMap2D_Response> response) -> void {
    (void)request_header;

    createCostMap2DLayers(map_);

    if (costmap_2d_layers_.find("obstacle_layer") != costmap_2d_layers_.end()) {
      response->costmap_2d = costmap_2d_layers_["obstacle_layer"];
    } else {
      response->costmap_2d = costmap_2d_layers_["inflation_layer"];
    }
    response->success = true;
    response->message = "Called /get_costmap_2d. Send map done.";
  };
  get_costmap_2d_srv_ =
    create_service<ike_nav_msgs::srv::GetCostMap2D>("get_costmap_2d", get_costmap_2d);

  auto publish_costmap_2d =
    [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger_Request> request,
      std::shared_ptr<std_srvs::srv::Trigger_Response> response) -> void {
    (void)request_header;

    createCostMap2DLayers(map_);
    publishCostMap2DLayers(costmap_2d_layers_);

    response->success = true;
    response->message = "Called /publish_costmap_2d. Send map done.";
  };
  publish_map_srv_ =
    create_service<std_srvs::srv::Trigger>("publish_costmap_2d", publish_costmap_2d);
}

void IkeCostMap2D::initServiceClient()
{
  get_map_srv_client_ = this->create_client<ike_nav_msgs::srv::GetMap>("get_map");
}

void IkeCostMap2D::initTimer()
{
  using namespace std::chrono_literals;

  create_obstacle_layer_timer_ = this->create_wall_timer(
    std::chrono::milliseconds{publish_costmap_2d_ms_},
    std::bind(&IkeCostMap2D::createObstacleLayer, this));
}

void IkeCostMap2D::getMap()
{
  while (!get_map_srv_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
  }

  auto request = std::make_shared<ike_nav_msgs::srv::GetMap::Request>();
  using ServiceResponseFuture = rclcpp::Client<ike_nav_msgs::srv::GetMap>::SharedFuture;

  auto response_received_callback = [this](ServiceResponseFuture future) {
    auto result = future.get();
    map_ = result.get()->map;
    get_map_ = true;

    createCostMap2DLayers(map_);
    publishCostMap2DLayers(costmap_2d_layers_);
  };
  auto future_result = get_map_srv_client_->async_send_request(request, response_received_callback);
}

void IkeCostMap2D::createCostMap2DLayers(const nav_msgs::msg::OccupancyGrid & map)
{
  costmap_2d_layers_["static_layer"] = createStaticLayer(map);
  costmap_2d_layers_["inflation_layer"] = createInflationLayer(map);
}

nav_msgs::msg::OccupancyGrid IkeCostMap2D::createStaticLayer(
  const nav_msgs::msg::OccupancyGrid & map)
{
  return map;
};

nav_msgs::msg::OccupancyGrid IkeCostMap2D::createInflationLayer(
  const nav_msgs::msg::OccupancyGrid & map)
{
  auto inflation_layer = map;

  for (uint32_t map_y = 0; map_y < inflation_layer.info.height; map_y++) {
    for (uint32_t map_x = 0; map_x < inflation_layer.info.width; map_x++)
      if (inflation_layer.data[map_y * inflation_layer.info.width + map_x] == 100) {
        calculateInflation(inflation_layer, inflation_layer_inflation_radius_, map_x, map_y);
      }
  }

  return inflation_layer;
};

void IkeCostMap2D::createObstacleLayer()
{
  if (costmap_2d_layers_.find("inflation_layer") != costmap_2d_layers_.end()) {
    costmap_2d_layers_["obstacle_layer"] = costmap_2d_layers_["inflation_layer"];

    auto lidar_pose = getMapFrameRobotPose();
    if (get_map_ && get_scan_ && get_lidar_pose_) {
      auto hists_xy = calculateHitPoint(scan_, lidar_pose);

      for (const auto & hit_xy : hists_xy) {
        // clang-format off
        costmap_2d_layers_
          ["obstacle_layer"].data[hit_xy.second * map_.info.width + hit_xy.first] 
            = 100;
        // clang-format on

        calculateInflation(
          costmap_2d_layers_["obstacle_layer"], obstacle_layer_inflation_radius_, hit_xy.first,
          hit_xy.second);

        costmap_2d_layers_["obstacle_layer"].header.stamp = rclcpp::Time();
        obstacle_layer_pub_->publish(costmap_2d_layers_["obstacle_layer"]);
        costmap_2d_pub_->publish(costmap_2d_layers_["obstacle_layer"]);
      }
    }
  }
}

std::vector<std::pair<uint32_t, uint32_t>> IkeCostMap2D::calculateHitPoint(
  sensor_msgs::msg::LaserScan scan, geometry_msgs::msg::PoseStamped lidar_pose)
{
  std::vector<std::pair<uint32_t, uint32_t>> hits_xy;
  double scan_angle_increment = scan.angle_min;
  for (auto scan_range : scan.ranges) {
    scan_angle_increment += scan_.angle_increment;
    if (
      std::isinf(scan_range) || std::isnan(scan_range) ||
      scan_range > obstacle_layer_obstacle_range_)
      continue;

    auto hit_x = lidar_pose.pose.position.x +
                 scan_range * cos(tf2::getYaw(lidar_pose.pose.orientation) + scan_angle_increment);

    auto hit_y = lidar_pose.pose.position.y +
                 scan_range * sin(tf2::getYaw(lidar_pose.pose.orientation) + scan_angle_increment);

    hits_xy.push_back(std::make_pair(
      static_cast<uint32_t>(hit_x / map_.info.resolution),
      static_cast<uint32_t>(hit_y / map_.info.resolution)));
  }

  return hits_xy;
}

geometry_msgs::msg::PoseStamped IkeCostMap2D::getMapFrameRobotPose()
{
  geometry_msgs::msg::PoseStamped lidar_pose;
  if (nav2_util::getCurrentPose(lidar_pose, *tf_buffer_)) {
    get_lidar_pose_ = true;
  }

  return lidar_pose;
}

void IkeCostMap2D::calculateInflation(
  nav_msgs::msg::OccupancyGrid & map, const double & inflation_radius, const uint32_t & map_x,
  const uint32_t & map_y)
{
  auto sub_map_x_start = map_x - inflation_radius;
  auto sub_map_y_start = map_y - inflation_radius;
  auto sub_map_x_end = map_x + inflation_radius;
  auto sub_map_y_end = map_y + inflation_radius;

  for (auto y = sub_map_y_start - inflation_radius; y < sub_map_y_end; y++) {
    for (auto x = sub_map_x_start - inflation_radius; x < sub_map_x_end; x++) {
      if (hypot(x - map_x, y - map_y) < inflation_radius) {
        try {
          map.data.at(y * map.info.width + x);
        } catch (const std::out_of_range & e) {
          continue;
        }

        if (
          normalizeCost(
            calculateCost(0., inflation_radius),
            calculateCost(hypot(x - map_x, y - map_y), inflation_radius)) >
          map.data[y * map.info.width + x]) {
          map.data[y * map.info.width + x] = normalizeCost(
            calculateCost(0., inflation_radius),
            calculateCost(hypot(x - map_x, y - map_y), inflation_radius));
        }
      }
    }
  }
}

double IkeCostMap2D::calculateCost(double stochastic_variable, double inflation_radius)
{
  double sigma = inflation_radius / 3.;
  double cost = 1. / std::sqrt(2. * M_PI * sigma * sigma) *
                std::exp(-stochastic_variable * stochastic_variable / (2. * sigma * sigma));
  return cost;
}

double IkeCostMap2D::normalizeCost(double max_pdf, double pdf) { return (pdf / max_pdf) * 100.; }

void IkeCostMap2D::publishCostMap2DLayers(
  std::map<std::string, nav_msgs::msg::OccupancyGrid> & costmap_2d_layers)
{
  costmap_2d_layers["static_layer"].header.stamp = rclcpp::Time();
  costmap_2d_layers["inflation_layer"].header.stamp = rclcpp::Time();

  static_layer_pub_->publish(costmap_2d_layers["static_layer"]);
  inflation_layer_pub_->publish(costmap_2d_layers["inflation_layer"]);

  if (get_map_ && get_scan_ && get_lidar_pose_) {
    costmap_2d_layers["obstacle_layer"].header.stamp = rclcpp::Time();
    obstacle_layer_pub_->publish(costmap_2d_layers["obstacle_layer"]);
  }
}

}  // namespace ike_nav

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ike_nav::IkeCostMap2D)
