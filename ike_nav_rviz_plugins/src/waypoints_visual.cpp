// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "waypoints_visual.hpp"

#include "rviz_rendering/objects/arrow.hpp"

#include <Ogre.h>

namespace ike_nav_rviz_plugins
{

// BEGIN_TUTORIAL
WaypointsVisual::WaypointsVisual(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node)
{
  scene_manager_ = scene_manager;

  frame_node_ = parent_node->createChildSceneNode();

  acceleration_arrow_.reset(new rviz_rendering::Arrow(scene_manager_, frame_node_));
}

WaypointsVisual::~WaypointsVisual() { scene_manager_->destroySceneNode(frame_node_); }

void WaypointsVisual::setMessage(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  const geometry_msgs::msg::Vector3 & a = msg->linear_acceleration;

  Ogre::Vector3 acc(static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(a.z));

  float length = acc.length();

  Ogre::Vector3 scale(length, length, length);
  acceleration_arrow_->setScale(scale);

  acceleration_arrow_->setDirection(acc);
}

void WaypointsVisual::setFramePosition(const Ogre::Vector3 & position)
{
  frame_node_->setPosition(position);
}

void WaypointsVisual::setFrameOrientation(const Ogre::Quaternion & orientation)
{
  frame_node_->setOrientation(orientation);
}

// Color is passed through to the Arrow object.
void WaypointsVisual::setColor(float r, float g, float b, float a)
{
  acceleration_arrow_->setColor(r, g, b, a);
}
// END_TUTORIAL

}  // namespace ike_nav_rviz_plugins
