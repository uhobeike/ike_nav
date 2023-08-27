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
  void updateColorAndAlpha();
  void updateScale();
  void updateYawOnlyOrientation();

private:
  void processMessage(ike_nav_msgs::msg::Waypoints::ConstSharedPtr msg) override;

  std::deque<std::shared_ptr<WaypointsVisual>> visuals_;
  std::size_t history_length_{1};

  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::FloatProperty * scale_property_;
  rviz_common::properties::FloatProperty * yaw_only_orientation_property_;
  rviz_common::properties::IntProperty * history_length_property_;
};

}  // namespace ike_nav_rviz_plugins

#endif  // WAYPOINTS_DISPLAY_HPP_
