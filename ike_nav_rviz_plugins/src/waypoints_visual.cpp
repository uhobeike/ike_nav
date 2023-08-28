// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "waypoints_visual.hpp"

#include "rviz_rendering/material_manager.hpp"
#include "rviz_rendering/objects/shape.hpp"

namespace ike_nav_rviz_plugins
{

WaypointsVisual::WaypointsVisual(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node)
{
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();

  // Waypoint Area
  waypoint_area_node_ = frame_node_->createChildSceneNode();
  waypoint_area_ = std::make_shared<rviz_rendering::Shape>(
    rviz_rendering::Shape::Cylinder, scene_manager, waypoint_area_node_);

  // Waypoint Flag
  waypoint_flag_resource_ = "package://ike_nav_rviz_plugins/media/ike_nav_goal_flag.dae";
  if (!rviz_rendering::loadMeshFromResource(waypoint_flag_resource_)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("waypoints_display"), "PlantFlagTool: failed to load model resource '%s'.",
      waypoint_flag_resource_.c_str());
    return;
  }
  waypoint_flag_node_ = frame_node_->createChildSceneNode();
  waypoint_flag_entity_ = scene_manager_->createEntity(waypoint_flag_resource_);
  waypoint_flag_node_->attachObject(waypoint_flag_entity_);
}

WaypointsVisual::~WaypointsVisual() { scene_manager_->destroySceneNode(frame_node_); }

void WaypointsVisual::setWaypointAreaPosition(const Ogre::Vector3 & position)
{
  waypoint_area_node_->setPosition(position);
}

void WaypointsVisual::setWaypointAreaOrientation(const Ogre::Quaternion & orientation)
{
  waypoint_area_node_->setOrientation(orientation);
}

void WaypointsVisual::setWaypointAreaColor(
  const float & r, const float & g, const float & b, const float & a)
{
  waypoint_area_->setColor(r, g, b, a);
}

void WaypointsVisual::setWaypointAreaScale(const Ogre::Vector3 & s)
{
  waypoint_area_node_->setScale(s);
}

void WaypointsVisual::setWaypointFlagPosition(const Ogre::Vector3 & position)
{
  waypoint_flag_node_->setPosition(position);
}

void WaypointsVisual::setWaypointFlagOrientation(const Ogre::Quaternion & orientation)
{
  waypoint_flag_node_->setOrientation(orientation);
}

void WaypointsVisual::setWaypointFlagColor(
  const float & r, const float & g, const float & b, const float & a)
{
  Ogre::String new_material_name =
    "goal_flag_material_" + Ogre::StringConverter::toString(Ogre::Math::UnitRandom());
  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
    new_material_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  material->setDiffuse(Ogre::ColourValue(r, g, b, a));
  waypoint_flag_entity_->setMaterial(material);

  rviz_rendering::MaterialManager::enableAlphaBlending(material, a);
}

void WaypointsVisual::setWaypointFlagScale(const float & scalse)
{
  waypoint_flag_node_->setScale(Ogre::Vector3(scalse, scalse, scalse));
}

}  // namespace ike_nav_rviz_plugins
