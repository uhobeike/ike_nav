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
    "Color", QColor(204, 51, 204), "Color to draw the acceleration arrows.", this,
    SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.", this,
    SLOT(updateColorAndAlpha()));

  history_length_property_ = new rviz_common::properties::IntProperty(
    "History Length", 1, "Number of prior measurements to display.", this,
    SLOT(updateHistoryLength()));
  history_length_property_->setMin(1);
  history_length_property_->setMax(100000);
}

void WaypointsDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateHistoryLength();
}

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

void WaypointsDisplay::updateHistoryLength()
{
  history_length_ = static_cast<std::size_t>(history_length_property_->getInt());
  if (visuals_.size() > history_length_) {
    visuals_.resize(history_length_);
  }
}

void WaypointsDisplay::processMessage(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
        msg->header.frame_id, msg->header.stamp, position, orientation)) {
    RCLCPP_INFO(
      rclcpp::get_logger("imu_display"), "Error transforming from frame '%s' to frame '%s'",
      msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  std::shared_ptr<WaypointsVisual> visual;
  visual.reset(new WaypointsVisual(context_->getSceneManager(), scene_node_));
  visual->setMessage(msg);
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);

  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  visual->setColor(color.r, color.g, color.b, alpha);

  if (visuals_.size() == history_length_) {
    visuals_.pop_back();
  }
  visuals_.push_front(visual);
}

}  // namespace ike_nav_rviz_plugins

PLUGINLIB_EXPORT_CLASS(ike_nav_rviz_plugins::WaypointsDisplay, rviz_common::Display)
