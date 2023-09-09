// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef IKE_NAV_RVIZ_PLUGINS__WAYPOINT_SET_TOOL_HPP_
#define IKE_NAV_RVIZ_PLUGINS__WAYPOINT_SET_TOOL_HPP_

#include "ike_waypoint_follower_parameter/ike_waypoint_follower_parameter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_rendering/objects/movable_text.hpp>

#include "ike_nav_msgs/msg/waypoints.hpp"
#include "ike_nav_msgs/srv/get_waypoints_msg.hpp"
#include <std_srvs/srv/trigger.hpp>

#include <Ogre.h>

#include <string>
#include <vector>

namespace Ogre
{
class SceneNode;
}

namespace rviz_common
{
class VisualizationManager;
class ViewportMouseEvent;
}  // namespace rviz_common

namespace ike_nav_rviz_plugins
{

class WaypointsVisual;

class WaypointSetTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  WaypointSetTool();
  ~WaypointSetTool();

protected:
  void getParam();
  void initPublisher();
  void initSubscription();
  void initServiceServer();

  void onInitialize() override;

  void activate() override;
  void deactivate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

  void moveWaypoint(const Ogre::Vector3 & position);
  bool getWaypointsProperty(const rviz_common::DisplayGroup * display_group);

  void makeWaypoints();
  ike_nav_msgs::msg::Waypoints createWaypointsMsg();
  void publishWaypoints(const ike_nav_msgs::msg::Waypoints & msg);

  rclcpp::Node::SharedPtr createNewNode(const std::string & node_name);

  void timerEvent(QTimerEvent * event);

private:
  u_int32_t waypoint_id_cnt_;

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Publisher<ike_nav_msgs::msg::Waypoints>::SharedPtr waypoints_pub_;
  rclcpp::Subscription<ike_nav_msgs::msg::Waypoints>::SharedPtr waypoints_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr delete_waypoint_service_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr delete_all_waypoints_service_server_;

  rclcpp::Service<ike_nav_msgs::srv::GetWaypointsMsg>::SharedPtr get_waypoints_msg_service_server_;

  std::shared_ptr<ike_waypoint_follower::ParamListener> param_listener_;
  ike_waypoint_follower::Params params_;

  std::shared_ptr<WaypointsVisual> visual_;

  Ogre::SceneNode * move_waypoint_node_;

  // Waypoint Area
  rviz_common::properties::ColorProperty * waypoint_area_color_property_;

  // Waypoint Flag
  rviz_common::properties::ColorProperty * waypoint_flag_color_property_;
  rviz_common::properties::Property * waypoint_flag_scale_property_;

  // Waypoint Text
  rviz_common::properties::ColorProperty * waypoint_text_color_property_;
  rviz_common::properties::Property * waypoint_text_scale_property_;

  // Waypoints
  rviz_common::properties::Property * waypoints_alpha_property_;

  bool get_waypoints_property_;

  std::deque<Ogre::Vector2> waypoints_position_;
  double waypoint_radius_;

  ike_nav_msgs::msg::Waypoints waypoints_;
  bool get_waypoints_;

  int timer_id_;
};

}  // namespace ike_nav_rviz_plugins

#endif  // IKE_NAV_RVIZ_PLUGINS__WAYPOINT_SET_TOOL_HPP_
