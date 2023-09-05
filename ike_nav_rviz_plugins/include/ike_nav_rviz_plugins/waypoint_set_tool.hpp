#ifndef PLANT_FLAG_TOOL_HPP_
#define PLANT_FLAG_TOOL_HPP_

#include "ike_waypoint_follower_parameter/ike_waypoint_follower_parameter.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_rendering/objects/movable_text.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/properties/color_property.hpp>

#include <ike_nav_msgs/msg/waypoints.hpp>

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

private:
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Publisher<ike_nav_msgs::msg::Waypoints>::SharedPtr waypoints_pub_;

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
  u_int32_t waypoint_id_cnt_;

  std::deque<Ogre::Vector2> waypoints_position_;

  double waypoint_radius_;
};

}  // namespace ike_nav_rviz_plugins

#endif
