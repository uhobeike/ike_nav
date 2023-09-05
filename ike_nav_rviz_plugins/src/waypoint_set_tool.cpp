
#include "ike_nav_rviz_plugins/waypoint_set_tool.hpp"

#include "ike_nav_rviz_plugins/waypoints_visual.hpp"
#include "ike_waypoint_follower_parameter/ike_waypoint_follower_parameter.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_rendering/geometry.hpp"
#include "rviz_rendering/mesh_loader.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"

#include <Ogre.h>

#include <memory>

namespace ike_nav_rviz_plugins
{

WaypointSetTool::WaypointSetTool()
: waypoint_id_cnt_(1), client_node_(createNewNode("waypoints_set_tool"))
{
  getParam();

  initPublisher();
  initServiceServer();

  timer_id_ = startTimer(100);
}

WaypointSetTool::~WaypointSetTool() {}

void WaypointSetTool::getParam()
{
  this->param_listener_ = std::make_shared<ike_waypoint_follower::ParamListener>(
    client_node_->get_node_parameters_interface());
  this->params_ = param_listener_->get_params();

  waypoint_radius_ = this->params_.waypoint_radius;
}

void WaypointSetTool::initPublisher()
{
  waypoints_pub_ = client_node_->create_publisher<ike_nav_msgs::msg::Waypoints>(
    "waypoints", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

void WaypointSetTool::initServiceServer()
{
  auto delete_waypoint =
    [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger_Request> request,
      std::shared_ptr<std_srvs::srv::Trigger_Response> response) -> void {
    (void)request_header;

    if (waypoint_id_cnt_ > 1 && waypoints_position_.size()) {
      waypoint_id_cnt_ -= 1;
      waypoints_position_.pop_back();
    }
    makeWaypoints();

    response->success = true;
    response->message = "Called /delete_waypoint. Send goal done.";
  };
  delete_waypoint_ =
    client_node_->create_service<std_srvs::srv::Trigger>("delete_waypoint", delete_waypoint);

  auto delete_all_waypoints =
    [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger_Request> request,
      std::shared_ptr<std_srvs::srv::Trigger_Response> response) -> void {
    (void)request_header;

    waypoint_id_cnt_ = 1;
    waypoints_position_.clear();
    makeWaypoints();

    response->success = true;
    response->message = "Called /delete_all_waypoints. Send goal done.";
  };
  delete_all_waypoints_ = client_node_->create_service<std_srvs::srv::Trigger>(
    "delete_all_waypoints", delete_all_waypoints);
}

void WaypointSetTool::onInitialize()
{
  move_waypoint_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  visual_ = std::make_shared<WaypointsVisual>(context_->getSceneManager(), move_waypoint_node_);
  move_waypoint_node_->setVisible(false);
}

void WaypointSetTool::activate()
{
  if (move_waypoint_node_) {
    get_waypoints_property_ = getWaypointsProperty(context_->getRootDisplayGroup());
    moveWaypoint(Ogre::Vector3{0., 0., 0.});
    move_waypoint_node_->setVisible(true);
  }
}

void WaypointSetTool::deactivate()
{
  if (move_waypoint_node_) {
    move_waypoint_node_->setVisible(false);
  }
}

int WaypointSetTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (!move_waypoint_node_) {
    return Render;
  }
  auto projection_finder = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
  auto projection = projection_finder->getViewportPointProjectionOnXYPlane(
    event.panel->getRenderWindow(), event.x, event.y);
  Ogre::Vector3 intersection = projection.second;
  if (projection.first) {
    moveWaypoint(intersection);
    move_waypoint_node_->setVisible(true);

    if (event.leftDown()) {
      ++waypoint_id_cnt_;
      waypoints_position_.push_back(Ogre::Vector2(intersection.x, intersection.y));
      makeWaypoints();
      return Render | Finished;
    }
  } else {
    move_waypoint_node_->setVisible(false);
  }
  return Render;
}

void WaypointSetTool::moveWaypoint(const Ogre::Vector3 & position)
{
  Ogre::Vector3 waypoint_flag_position;
  waypoint_flag_position.x = position.x;
  waypoint_flag_position.y = position.y;
  waypoint_flag_position.z = position.z;

  Ogre::Vector3 waypoint_area_position;
  waypoint_area_position.x = position.x;
  waypoint_area_position.y = position.y;
  waypoint_area_position.z = 0.01;

  Ogre::Vector3 waypoint_text_position;
  waypoint_text_position.x = position.x;
  waypoint_text_position.y = position.y;
  waypoint_text_position.z = 1. * 2. + 0.1;

  Ogre::Vector3 waypoint_area_scale;
  waypoint_area_scale.x = 0.5 * 2.;
  waypoint_area_scale.y = 0.01;
  waypoint_area_scale.z = 0.5 * 2.;

  visual_->setWaypointAreaPosition(waypoint_area_position);
  visual_->setWaypointFlagPosition(waypoint_flag_position);
  visual_->setWaypointTextPosition(waypoint_text_position);
  visual_->setWaypointAreaScale(waypoint_area_scale);
  visual_->setWaypointTextCaption(Ogre::String(std::to_string(waypoint_id_cnt_)));
  visual_->setWaypointTextHeight(0.3);

  Ogre::Quaternion waypoint_area_orientation =
    Ogre::Quaternion(Ogre::Radian(M_PI / 2.), Ogre::Vector3::UNIT_X);
  visual_->setWaypointAreaOrientation(waypoint_area_orientation);

  if (get_waypoints_property_) {
    Ogre::Vector3 waypoint_text_position;
    waypoint_text_position.x = position.x;
    waypoint_text_position.y = position.y;
    waypoint_text_position.z = waypoint_flag_scale_property_->getValue().toFloat() * 2. + 0.1;

    visual_->setWaypointTextPosition(waypoint_text_position);
    visual_->setWaypointTextHeight(waypoint_text_scale_property_->getValue().toFloat());
    visual_->setWaypointFlagScale(waypoint_flag_scale_property_->getValue().toFloat());

    float alpha = waypoints_alpha_property_->getValue().toFloat();
    Ogre::ColourValue text_color = waypoint_text_color_property_->getOgreColor();
    visual_->setWaypointTextColor(text_color.r, text_color.g, text_color.b, alpha);
    Ogre::ColourValue flag_color = waypoint_flag_color_property_->getOgreColor();
    visual_->setWaypointFlagColor(flag_color.r, flag_color.g, flag_color.b, alpha);
    Ogre::ColourValue area_color = waypoint_area_color_property_->getOgreColor();
    visual_->setWaypointAreaColor(area_color.r, area_color.g, area_color.b, alpha);
  }
}

bool WaypointSetTool::getWaypointsProperty(const rviz_common::DisplayGroup * display_group)
{
  for (int diplay_number = 0; diplay_number < display_group->numDisplays(); ++diplay_number) {
    rviz_common::DisplayGroup * sub_display =
      qobject_cast<rviz_common::DisplayGroup *>(display_group->getDisplayAt(diplay_number));
    if (sub_display) {
      if (getWaypointsProperty(sub_display)) {
        return true;
      }
    } else {
      if (display_group->getDisplayAt(diplay_number)->getNameStd() == "Waypoints") {
        // clang-format off
        waypoint_text_color_property_ = dynamic_cast<rviz_common::properties::ColorProperty *>
                                          (display_group->getDisplayAt(diplay_number)
                                          ->subProp("Waypoint Text Color"));
        waypoint_area_color_property_ = dynamic_cast<rviz_common::properties::ColorProperty *>
                                          (display_group->getDisplayAt(diplay_number)
                                          ->subProp("Waypoint Area Color"));
        waypoint_flag_color_property_ = dynamic_cast<rviz_common::properties::ColorProperty *>
                                          (display_group->getDisplayAt(diplay_number)
                                          ->subProp("Waypoint Flag Color"));
        waypoint_flag_scale_property_ = display_group->getDisplayAt(diplay_number)
                                          ->subProp("Waypoint Flag Scale");
        waypoint_text_scale_property_ = display_group->getDisplayAt(diplay_number)
                                          ->subProp("Waypoint Text Scale");
        waypoints_alpha_property_ = display_group->getDisplayAt(diplay_number)
                                          ->subProp("Waypoints Alpha");
        // clang-format on
        return true;
      }
    }
  }

  return false;
}

void WaypointSetTool::makeWaypoints()
{
  auto waypoints_msg = createWaypointsMsg();
  publishWaypoints(waypoints_msg);
}

ike_nav_msgs::msg::Waypoints WaypointSetTool::createWaypointsMsg()
{
  ike_nav_msgs::msg::Waypoints waypoints_msg;
  for (uint32_t i = 0; i < waypoints_position_.size(); ++i) {
    ike_nav_msgs::msg::Waypoint waypoint_msg;
    waypoint_msg.id = i + 1;
    waypoint_msg.pose.position.x = waypoints_position_[i].x;
    waypoint_msg.pose.position.y = waypoints_position_[i].y;

    waypoint_msg.function.variable_waypoint_radius.waypoint_radius = waypoint_radius_;

    waypoints_msg.waypoints.push_back(waypoint_msg);
  }

  waypoints_msg.header.frame_id = "map";
  waypoints_msg.header.stamp = rclcpp::Time();

  return waypoints_msg;
}
void WaypointSetTool::publishWaypoints(const ike_nav_msgs::msg::Waypoints & msg)
{
  waypoints_pub_->publish(msg);
}

rclcpp::Node::SharedPtr WaypointSetTool::createNewNode(const std::string & node_name)
{
  std::string node = "__node:=" + node_name;
  auto options = rclcpp::NodeOptions().arguments({"--ros-args", "--remap", node, "--"});
  return std::make_shared<rclcpp::Node>("_", options);
}

void WaypointSetTool::timerEvent(QTimerEvent * event)
{
  if (event->timerId() == timer_id_) rclcpp::spin_some(client_node_);
}

}  // namespace ike_nav_rviz_plugins

PLUGINLIB_EXPORT_CLASS(ike_nav_rviz_plugins::WaypointSetTool, rviz_common::Tool)
