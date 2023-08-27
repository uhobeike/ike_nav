// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef WAYPOINTS_VISUAL_HPP_
#define WAYPOINTS_VISUAL_HPP_

#include "rviz_rendering/geometry.hpp"
#include "rviz_rendering/mesh_loader.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"

#include <rclcpp/rclcpp.hpp>

#include <Ogre.h>

#include <memory>

namespace rviz_rendering
{
class Shape;
}

namespace ike_nav_rviz_plugins
{
class WaypointsVisual
{
public:
  WaypointsVisual(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node);

  virtual ~WaypointsVisual();

  // Waypoint Area
  void setWaypointAreaPosition(const Ogre::Vector3 & position);
  void setWaypointAreaOrientation(const Ogre::Quaternion & orientation);
  void setWaypointAreaColor(const float & r, const float & g, const float & b, const float & a);
  void setWaypointAreaScale(const Ogre::Vector3 & s);

  // Waypoint Flag
  void setWaypointFlagPosition(const Ogre::Vector3 & position);
  void setWaypointFlagOrientation(const Ogre::Quaternion & orientation);
  void setWaypointFlagColor(const float & r, const float & g, const float & b, const float & a);
  void setWaypointFlagScale(const float & scalse);

private:
  Ogre::SceneNode * frame_node_;
  Ogre::SceneNode * waypoint_area_node_;
  Ogre::SceneNode * waypoint_flag_node_;

  Ogre::SceneManager * scene_manager_;
  Ogre::Entity * waypoint_flag_entity_;

  std::shared_ptr<rviz_rendering::Shape> waypoint_area_;

  std::string waypoint_flag_resource_;
};

}  // namespace ike_nav_rviz_plugins

#endif  // WAYPOINTS_VISUAL_HPP_
