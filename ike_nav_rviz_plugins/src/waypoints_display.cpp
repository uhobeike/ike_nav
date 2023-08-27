// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "waypoints_display.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "tf2_ros/transform_listener.h"
#include "waypoints_visual.hpp"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <memory>

namespace ike_nav_rviz_plugins
{

WaypointsDisplay::WaypointsDisplay()
{
  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(0, 0, 0), "Color to draw the waypoints.", this, SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.", this,
    SLOT(updateColorAndAlpha()));
  alpha_property_->setMin(0.);
  alpha_property_->setMax(1.);

  scale_property_ = new rviz_common::properties::FloatProperty(
    "Scale", 1.0, "change waypoints size.", this, SLOT(updateScale()));
  alpha_property_->setMin(0.);

  yaw_only_orientation_property_ = new rviz_common::properties::FloatProperty(
    "Yaw", 0.0, "change only yaw direction of the waypoint.", this,
    SLOT(updateYawOnlyOrientation()));
  yaw_only_orientation_property_->setMin(-1. * M_PI);
  yaw_only_orientation_property_->setMax(M_PI);
}

void WaypointsDisplay::onInitialize() { MFDClass::onInitialize(); }

WaypointsDisplay::~WaypointsDisplay() {}

void WaypointsDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

void WaypointsDisplay::updateColorAndAlpha()
{
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_.size(); i++) {
    visuals_[i]->setColor(color.r, color.g, color.b, alpha);
  }
}

void WaypointsDisplay::updateScale()
{
  float scale = scale_property_->getFloat();
  for (size_t i = 0; i < visuals_.size(); i++) {
    visuals_[i]->setScale(scale);
  }
}

void WaypointsDisplay::updateYawOnlyOrientation()
{
  float yaw = yaw_only_orientation_property_->getFloat();
  Ogre::Quaternion yawOnlyOrientation = Ogre::Quaternion(Ogre::Radian(yaw), Ogre::Vector3::UNIT_Z);
  for (size_t i = 0; i < visuals_.size(); i++) {
    visuals_[i]->setOrientation(yawOnlyOrientation);
  }
}

void WaypointsDisplay::processMessage(ike_nav_msgs::msg::Waypoints::ConstSharedPtr msg)
{
  visuals_.clear();

  for (const auto & waypoint : msg->waypoints) {
    Ogre::Vector3 position;
    position.x = waypoint.pose.position.x;
    position.y = waypoint.pose.position.y;
    position.z = waypoint.pose.position.z;

    std::shared_ptr<WaypointsVisual> visual;
    visual.reset(new WaypointsVisual(context_->getSceneManager(), scene_node_));
    visual->setPosition(position);

    float alpha = alpha_property_->getFloat();
    Ogre::ColourValue color = color_property_->getOgreColor();
    visual->setColor(color.r, color.g, color.b, alpha);

    visuals_.push_front(visual);
  }

  updateColorAndAlpha();
  updateScale();
  updateYawOnlyOrientation();
}

}  // namespace ike_nav_rviz_plugins

PLUGINLIB_EXPORT_CLASS(ike_nav_rviz_plugins::WaypointsDisplay, rviz_common::Display)
