// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "ike_nav_rviz_plugins/waypoints_display.hpp"

#include "ike_nav_rviz_plugins/waypoints_visual.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/visualization_manager.hpp>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

namespace ike_nav_rviz_plugins
{

WaypointsDisplay::WaypointsDisplay()
{
  waypoint_text_color_property_ = new rviz_common::properties::ColorProperty(
    "Waypoint Text Color", QColor(0, 0, 0), "Color to draw the waypoints text.", this,
    SLOT(updateWaypointTextColorAndAlpha()));

  waypoint_area_color_property_ = new rviz_common::properties::ColorProperty(
    "Waypoint Area Color", QColor(0, 0, 0), "Color to draw the waypoints area.", this,
    SLOT(updateWaypointAreaColorAndAlpha()));

  waypoint_flag_color_property_ = new rviz_common::properties::ColorProperty(
    "Waypoint Flag Color", QColor(0, 0, 0), "Color to draw the waypoints flag.", this,
    SLOT(updateWaypointFlagColorAndAlpha()));

  waypoint_flag_yaw_only_orientation_property_ = new rviz_common::properties::FloatProperty(
    "Waypoint Flag Yaw", 0.0, "change only yaw direction of the waypoints flag.", this,
    SLOT(updateWaypointFlagYawOnlyOrientation()));
  waypoint_flag_yaw_only_orientation_property_->setMin(-1. * M_PI);
  waypoint_flag_yaw_only_orientation_property_->setMax(M_PI);

  waypoint_flag_scale_property_ = new rviz_common::properties::FloatProperty(
    "Waypoint Flag Scale", 1.0, "change waypoints size.", this, SLOT(updateWaypointFlagScale()));
  waypoint_flag_scale_property_->setMin(0.);

  waypoint_text_scale_property_ = new rviz_common::properties::FloatProperty(
    "Waypoint Text Scale", 1.0, "change waypoints text size.", this,
    SLOT(updateWaypointTextScale()));
  waypoint_flag_scale_property_->setMin(0.);

  waypoints_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Waypoints Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.", this,
    SLOT(updateWaypointsColorAndAlpha()));
  waypoints_alpha_property_->setMin(0.);
  waypoints_alpha_property_->setMax(1.);
}

void WaypointsDisplay::onInitialize() { MFDClass::onInitialize(); }

WaypointsDisplay::~WaypointsDisplay() {}

void WaypointsDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

void WaypointsDisplay::updateWaypointAreaColorAndAlpha()
{
  float alpha = waypoints_alpha_property_->getFloat();
  Ogre::ColourValue color = waypoint_area_color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_.size(); i++) {
    visuals_[i]->setWaypointAreaColor(color.r, color.g, color.b, alpha);
  }
}

void WaypointsDisplay::updateWaypointAreaScale(const Ogre::Vector3 & scale)
{
  for (size_t i = 0; i < visuals_.size(); i++) {
    visuals_[i]->setWaypointAreaScale(scale);
  }
}

void WaypointsDisplay::updateWaypointAreaOrientation(const Ogre::Quaternion & orientation)
{
  for (size_t i = 0; i < visuals_.size(); i++) {
    visuals_[i]->setWaypointAreaOrientation(orientation);
  }
}

void WaypointsDisplay::updateWaypointFlagColorAndAlpha()
{
  float alpha = waypoints_alpha_property_->getFloat();
  Ogre::ColourValue color = waypoint_flag_color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_.size(); i++) {
    visuals_[i]->setWaypointFlagColor(color.r, color.g, color.b, alpha);
  }
}

void WaypointsDisplay::updateWaypointFlagScale()
{
  float scale = waypoint_flag_scale_property_->getFloat();
  for (size_t i = 0; i < visuals_.size(); i++) {
    visuals_[i]->setWaypointFlagScale(scale);
    visuals_[i]->setWaypointTextPosition(Ogre::Vector3(
      waypoints_position_[i].x, waypoints_position_[i].y,
      waypoint_flag_scale_property_->getFloat() * 2. + 0.1));
  }
}

void WaypointsDisplay::updateWaypointFlagYawOnlyOrientation()
{
  float yaw = waypoint_flag_yaw_only_orientation_property_->getFloat();
  Ogre::Quaternion yaw_only_orientation =
    Ogre::Quaternion(Ogre::Radian(yaw), Ogre::Vector3::UNIT_Z);
  for (size_t i = 0; i < visuals_.size(); i++) {
    visuals_[i]->setWaypointFlagOrientation(yaw_only_orientation);
  }
}

void WaypointsDisplay::updateWaypointTextColorAndAlpha()
{
  float alpha = waypoints_alpha_property_->getFloat();
  Ogre::ColourValue color = waypoint_text_color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_.size(); i++) {
    visuals_[i]->setWaypointTextColor(color.r, color.g, color.b, alpha);
  }
}

void WaypointsDisplay::updateWaypointTextScale()
{
  float scale = waypoint_text_scale_property_->getFloat();
  for (size_t i = 0; i < visuals_.size(); i++) {
    visuals_[i]->setWaypointTextHeight(scale);
  }
}

void WaypointsDisplay::updateWaypointsColorAndAlpha()
{
  updateWaypointAreaColorAndAlpha();
  updateWaypointFlagColorAndAlpha();
  updateWaypointTextColorAndAlpha();
}

void WaypointsDisplay::processMessage(ike_nav_msgs::msg::Waypoints::ConstSharedPtr msg)
{
  visuals_.clear();
  waypoints_position_.clear();

  for (const auto & waypoint : msg->waypoints) {
    Ogre::Vector3 waypoint_flag_position;
    waypoint_flag_position.x = waypoint.pose.position.x;
    waypoint_flag_position.y = waypoint.pose.position.y;
    waypoint_flag_position.z = waypoint.pose.position.z;

    Ogre::Vector3 waypoint_area_position;
    waypoint_area_position.x = waypoint.pose.position.x;
    waypoint_area_position.y = waypoint.pose.position.y;
    waypoint_area_position.z = 0.01;

    Ogre::Vector3 waypoint_text_position;
    waypoint_text_position.x = waypoint.pose.position.x;
    waypoint_text_position.y = waypoint.pose.position.y;
    waypoint_text_position.z = waypoint_flag_scale_property_->getFloat() * 2. + 0.1;

    Ogre::Vector3 waypoint_area_scale;
    waypoint_area_scale.x = waypoint.function.variable_waypoint_radius.waypoint_radius * 2.;
    waypoint_area_scale.y = 0.01;
    waypoint_area_scale.z = waypoint.function.variable_waypoint_radius.waypoint_radius * 2.;

    std::shared_ptr<WaypointsVisual> visual;
    visual.reset(new WaypointsVisual(context_->getSceneManager(), scene_node_));
    visual->setWaypointAreaPosition(waypoint_area_position);
    visual->setWaypointFlagPosition(waypoint_flag_position);
    visual->setWaypointTextPosition(waypoint_text_position);
    visual->setWaypointAreaScale(waypoint_area_scale);
    visual->setWaypointTextCaption(Ogre::String(std::to_string(waypoint.id)));
    visual->setWaypointTextHeight(0.3);

    Ogre::Quaternion waypoint_area_orientation =
      Ogre::Quaternion(Ogre::Radian(M_PI / 2.), Ogre::Vector3::UNIT_X);
    visual->setWaypointAreaOrientation(waypoint_area_orientation);

    waypoints_position_.push_front(
      Ogre::Vector2(waypoint.pose.position.x, waypoint.pose.position.y));
    visuals_.push_front(visual);
  }

  updateWaypointAreaColorAndAlpha();
  updateWaypointFlagColorAndAlpha();
  updateWaypointTextColorAndAlpha();
  updateWaypointsColorAndAlpha();
  updateWaypointFlagScale();
  updateWaypointTextScale();
  updateWaypointFlagYawOnlyOrientation();
}

}  // namespace ike_nav_rviz_plugins

PLUGINLIB_EXPORT_CLASS(ike_nav_rviz_plugins::WaypointsDisplay, rviz_common::Display)
