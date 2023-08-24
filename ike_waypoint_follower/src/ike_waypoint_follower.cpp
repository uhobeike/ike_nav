// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "ike_waypoint_follower/ike_waypoint_follower.hpp"

#include <nav2_util/robot_utils.hpp>

#include <yaml-cpp/yaml.h>

namespace ike_nav
{

IkeWaypointFollower::IkeWaypointFollower(const rclcpp::NodeOptions & options)
: Node("ike_waypoint_follower", options)
{
  getParam();

  initTf();
  initActionClient();
  initTimer();

  readWaypointYaml();
}

void IkeWaypointFollower::getParam()
{
  this->param_listener_ =
    std::make_shared<ike_waypoint_follower::ParamListener>(this->get_node_parameters_interface());
  this->params_ = param_listener_->get_params();

  waypoint_yaml_path_ = this->params_.waypoint_yaml_path;
}

void IkeWaypointFollower::initTf()
{
  tf_buffer_.reset();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  // tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void IkeWaypointFollower::initActionClient()
{
  navigate_to_goal_action_client_ = rclcpp_action::create_client<NavigateToGoal>(
    this->get_node_base_interface(), this->get_node_graph_interface(),
    this->get_node_logging_interface(), this->get_node_waitables_interface(), "navigate_to_goal");
}

void IkeWaypointFollower::initTimer()
{
  using namespace std::chrono_literals;

  loop_timer_ = this->create_wall_timer(
    std::chrono::milliseconds{100ms}, std::bind(&IkeWaypointFollower::loop, this));
}

void IkeWaypointFollower::readWaypointYaml()
{
  YAML::Node waypoints_yaml = YAML::LoadFile(waypoint_yaml_path_);

  waypoints_.header.frame_id = "map";
  waypoints_.header.stamp = rclcpp::Time();
  waypoints_.waypoints.clear();

  if (!waypoints_yaml["waypoints"].IsNull()) {
    for (const auto & waypoint_yaml : waypoints_yaml["waypoints"]) {
      ike_nav_msgs::msg::Waypoint waypoint;

      waypoint.id = waypoint_yaml["id"].as<int32_t>();
      waypoint.pose.position.x = waypoint_yaml["position"]["x"].as<double>();
      waypoint.pose.position.y = waypoint_yaml["position"]["y"].as<double>();
      waypoint.pose.orientation.w = cos(waypoint_yaml["euler_angle"]["z"].as<double>() / 2.);
      waypoint.pose.orientation.z = sin(waypoint_yaml["euler_angle"]["z"].as<double>() / 2.);

      for (const auto & function : waypoint_yaml["functions"]) {
        if (function["function"].as<std::string>() == "variable_waypoint_radius") {
          if (!function["waypoint_radius"].IsNull()) {
            waypoint.function.variable_waypoint_radius.waypoint_radius =
              function["waypoint_radius"].as<float>();
          }
        }
      }

      waypoints_.waypoints.push_back(waypoint);
    }
  }
}

void IkeWaypointFollower::getMapFrameRobotPose(
  geometry_msgs::msg::PoseStamped & map_frame_robot_pose)
{
  geometry_msgs::msg::PoseStamped pose;
  if (nav2_util::getCurrentPose(pose, *tf_buffer_)) {
    map_frame_robot_pose = pose;
    get_robot_pose_ = true;
  }
}

void IkeWaypointFollower::loop()
{
  RCLCPP_INFO(get_logger(), "Run IkeWaypointFollower::loop");

  getMapFrameRobotPose(robot_pose_);
}

}  // namespace ike_nav

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ike_nav::IkeWaypointFollower)
