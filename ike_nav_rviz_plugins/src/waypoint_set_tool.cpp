
#include "ike_nav_rviz_plugins/waypoint_set_tool.hpp"

#include "ike_nav_rviz_plugins/waypoints_visual.hpp"
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

WaypointSetTool::WaypointSetTool() {}

WaypointSetTool::~WaypointSetTool() {}

void WaypointSetTool::onInitialize()
{
  move_waypoint_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  visual_ = std::make_shared<WaypointsVisual>(context_->getSceneManager(), move_waypoint_node_);
  move_waypoint_node_->setVisible(false);
}

void WaypointSetTool::activate()
{
  if (move_waypoint_node_) {
    get_waypoints_property_ = getWaypointsProperty();
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
      // makeWaypoints(intersection);
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
  // visual_->setWaypointTextCaption(Ogre::String(std::to_string(waypoint.id)));
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

bool WaypointSetTool::getWaypointsProperty()
{
  for (int diplay_number = 0; diplay_number < context_->getRootDisplayGroup()->numDisplays();
       ++diplay_number) {
    if (context_->getRootDisplayGroup()->getDisplayAt(diplay_number)->getNameStd() == "Waypoints") {
      // clang-format off
      waypoint_text_color_property_ = dynamic_cast<rviz_common::properties::ColorProperty *>
                                        (context_->getRootDisplayGroup()
                                        ->getDisplayAt(diplay_number)
                                        ->subProp("Waypoint Text Color"));
      waypoint_area_color_property_ = dynamic_cast<rviz_common::properties::ColorProperty *>
                                        (context_->getRootDisplayGroup()
                                        ->getDisplayAt(diplay_number)
                                        ->subProp("Waypoint Area Color"));
      waypoint_flag_color_property_ = dynamic_cast<rviz_common::properties::ColorProperty *>
                                        (context_->getRootDisplayGroup()
                                        ->getDisplayAt(diplay_number)
                                        ->subProp("Waypoint Flag Color"));
      waypoint_flag_scale_property_ = context_->getRootDisplayGroup()
                                        ->getDisplayAt(diplay_number)
                                        ->subProp("Waypoint Flag Scale");
      waypoint_text_scale_property_ = context_->getRootDisplayGroup()
                                        ->getDisplayAt(diplay_number)
                                        ->subProp("Waypoint Text Scale");
      waypoints_alpha_property_ = context_->getRootDisplayGroup()
                                        ->getDisplayAt(diplay_number)
                                        ->subProp("Waypoints Alpha");
      // clang-format on

      return true;
    }
  }

  return false;
}

void WaypointSetTool::makeWaypoints(const Ogre::Vector3 & position)
{
  createWaypointsMsg();
  publishWaypoints();
}
void WaypointSetTool::createWaypointsMsg() {}
void WaypointSetTool::publishWaypoints() {}

}  // namespace ike_nav_rviz_plugins

PLUGINLIB_EXPORT_CLASS(ike_nav_rviz_plugins::WaypointSetTool, rviz_common::Tool)
