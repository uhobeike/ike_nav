// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef WAYPOINTS_DISPLAY_HPP_
#define WAYPOINTS_DISPLAY_HPP_

#ifndef Q_MOC_RUN
#include "rviz_common/message_filter_display.hpp"

#include "ike_nav_msgs/msg/waypoints.hpp"

#include <deque>
#include <memory>
#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz_common::properties
{
class ColorProperty;
class FloatProperty;
class IntProperty;
}  // namespace rviz_common::properties

namespace ike_nav_rviz_plugins
{

class WaypointsVisual;

class WaypointsDisplay : public rviz_common::MessageFilterDisplay<ike_nav_msgs::msg::Waypoints>
{
  Q_OBJECT

public:
  WaypointsDisplay();
  virtual ~WaypointsDisplay();

protected:
  virtual void onInitialize();

  virtual void reset();

private Q_SLOTS:
  // Waypoint Area
  void updateWaypointAreaColorAndAlpha();
  void updateWaypointAreaScale(const Ogre::Vector3 & scale);
  void updateWaypointAreaOrientation(const Ogre::Quaternion & orientation);

  // Waypoint Flag
  void updateWaypointFlagColorAndAlpha();
  void updateWaypointFlagScale();
  void updateWaypointFlagYawOnlyOrientation();

  // Waypoint Text
  void updateWaypointTextColorAndAlpha();
  void updateWaypointTextScale();

  // Waypoints
  void updateWaypointsColorAndAlpha();

private:
  void processMessage(ike_nav_msgs::msg::Waypoints::ConstSharedPtr msg) override;

  std::deque<std::shared_ptr<WaypointsVisual>> visuals_;
  std::deque<Ogre::Vector2> waypoints_position_;

  // Waypoint Area
  rviz_common::properties::ColorProperty * waypoint_area_color_property_;

  // Waypoint Flag
  rviz_common::properties::ColorProperty * waypoint_flag_color_property_;
  rviz_common::properties::FloatProperty * waypoint_flag_scale_property_;
  rviz_common::properties::FloatProperty * waypoint_flag_yaw_only_orientation_property_;

  // Waypoint Text
  rviz_common::properties::ColorProperty * waypoint_text_color_property_;
  rviz_common::properties::FloatProperty * waypoint_text_scale_property_;

  // Waypoints
  rviz_common::properties::FloatProperty * waypoints_alpha_property_;
};

}  // namespace ike_nav_rviz_plugins

#endif  // WAYPOINTS_DISPLAY_HPP_
