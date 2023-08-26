// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "waypoints_visual.hpp"

#include <Ogre.h>
#include <OgreMaterialManager.h>

namespace ike_nav_rviz_plugins
{

WaypointsVisual::WaypointsVisual(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node)
{
  scene_manager_ = scene_manager;
  ogre_node_ = parent_node->createChildSceneNode();

  goal_flag_resource_ = "package://ike_nav_rviz_plugins/media/ike_nav_goal_flag.dae";

  if (!rviz_rendering::loadMeshFromResource(goal_flag_resource_)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("plant_flag_tool"), "PlantFlagTool: failed to load model resource '%s'.",
      goal_flag_resource_.c_str());
    return;
  }

  entity_ = scene_manager_->createEntity(goal_flag_resource_);
  ogre_node_->attachObject(entity_);
}

WaypointsVisual::~WaypointsVisual() { scene_manager_->destroySceneNode(ogre_node_); }

void WaypointsVisual::setMeshPose(const Ogre::Vector3 & position)
{
  ogre_node_->setPosition(position);
}

void WaypointsVisual::setColor(float r, float g, float b, float a)
{
  Ogre::String new_material_name =
    "goal_flag_material_" + Ogre::StringConverter::toString(Ogre::Math::UnitRandom());
  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
    new_material_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setDiffuse(Ogre::ColourValue(r, g, b, a));
  entity_->setMaterial(material);
}
// END_TUTORIAL

}  // namespace ike_nav_rviz_plugins
