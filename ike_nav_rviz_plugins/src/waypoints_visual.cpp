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
  ogre_node_ = parent_node->createChildSceneNode();

  goal_flag_resource_ = "package://ike_nav_rviz_plugins/media/ike_nav_goal_flag.dae";

  if (!rviz_rendering::loadMeshFromResource(goal_flag_resource_)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("waypoints_display"), "PlantFlagTool: failed to load model resource '%s'.",
      goal_flag_resource_.c_str());
    return;
  }

  entity_ = scene_manager_->createEntity(goal_flag_resource_);
  ogre_node_->attachObject(entity_);

  waypoint_area_ = std::make_shared<rviz_rendering::Shape>(
    rviz_rendering::Shape::Cylinder, scene_manager, ogre_node_);
}

WaypointsVisual::~WaypointsVisual() { scene_manager_->destroySceneNode(ogre_node_); }

void WaypointsVisual::setPosition(const Ogre::Vector3 & position)
{
  ogre_node_->setPosition(position);
}

void WaypointsVisual::setOrientation(const Ogre::Quaternion & orientation)
{
  Ogre::Quaternion yawOnlyOrientation = Ogre::Quaternion(Ogre::Radian(1.57), Ogre::Vector3::UNIT_Y);

  ogre_node_->setOrientation(yawOnlyOrientation);

  // waypoint_area_->setOrientation(yawOnlyOrientation);
}

void WaypointsVisual::setColor(float r, float g, float b, float a)
{
  Ogre::String new_material_name =
    "goal_flag_material_" + Ogre::StringConverter::toString(Ogre::Math::UnitRandom());
  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
    new_material_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  material->setDiffuse(Ogre::ColourValue(r, g, b, a));
  entity_->setMaterial(material);

  rviz_rendering::MaterialManager::enableAlphaBlending(material, a);

  waypoint_area_->setColor(0, 0, 1, 1);
}

void WaypointsVisual::setScale(float s)
{
  ogre_node_->setScale(Ogre::Vector3(s, s, s));

  waypoint_area_->setScale(Ogre::Vector3(1, 1, 1));
}

}  // namespace ike_nav_rviz_plugins
