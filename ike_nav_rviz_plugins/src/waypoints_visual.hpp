// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef WAYPOINTS_VISUAL_HPP_
#define WAYPOINTS_VISUAL_HPP_

#include "rviz_rendering/objects/arrow.hpp"

#include "sensor_msgs/msg/imu.hpp"

#include <Ogre.h>

#include <memory>

namespace Ogre
{
class Quaternion;
}

namespace rviz_rendering
{
class Arrow;
}

namespace ike_nav_rviz_plugins
{
class WaypointsVisual
{
public:
  WaypointsVisual(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node);

  virtual ~WaypointsVisual();

  void setMessage(sensor_msgs::msg::Imu::ConstSharedPtr msg);

  void setFramePosition(const Ogre::Vector3 & position);
  void setFrameOrientation(const Ogre::Quaternion & orientation);

  void setColor(float r, float g, float b, float a);

private:
  std::shared_ptr<rviz_rendering::Arrow> acceleration_arrow_;

  Ogre::SceneNode * frame_node_;

  Ogre::SceneManager * scene_manager_;
};

}  // namespace ike_nav_rviz_plugins

#endif  // WAYPOINTS_VISUAL_HPP_
