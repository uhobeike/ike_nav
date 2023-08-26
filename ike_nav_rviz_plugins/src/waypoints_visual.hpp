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

namespace ike_nav_rviz_plugins
{
class WaypointsVisual
{
public:
  WaypointsVisual(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node);

  virtual ~WaypointsVisual();

  void setMeshPose(const Ogre::Vector3 & position);

  void setColor(float r, float g, float b, float a);

private:
  Ogre::SceneNode * ogre_node_;

  Ogre::SceneManager * scene_manager_;

  Ogre::Entity * entity_;

  std::string goal_flag_resource_;
};

}  // namespace ike_nav_rviz_plugins

#endif  // WAYPOINTS_VISUAL_HPP_
