#ifndef PLANT_FLAG_TOOL_HPP_
#define PLANT_FLAG_TOOL_HPP_

#include "rviz_common/tool.hpp"
#include "rviz_rendering/objects/movable_text.hpp"

#include <Ogre.h>

#include <string>
#include <vector>

namespace Ogre
{
class SceneNode;
}

namespace rviz_common::properties
{
class VectorProperty;
}

namespace rviz_common
{
class VisualizationManager;
class ViewportMouseEvent;
}  // namespace rviz_common

namespace ike_nav_rviz_plugins
{

class WaypointSetTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  WaypointSetTool();
  ~WaypointSetTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent(rviz_common::ViewportMouseEvent & event);

  virtual void save(rviz_common::Config config) const;

private:
  void makeFlag(const Ogre::Vector3 & position);
  void makeText(const Ogre::Vector3 & position);

  std::vector<Ogre::SceneNode *> flag_nodes_;
  Ogre::SceneNode * moving_flag_node_;
  std::string flag_resource_;
  rviz_common::properties::VectorProperty * current_flag_property_;

  std::vector<Ogre::SceneNode *> text_nodes_;
  Ogre::SceneNode * moving_text_node_;
  std::vector<std::shared_ptr<rviz_rendering::MovableText>> waypoint_text_;

  uint32_t waypoint_id_;
};

}  // namespace ike_nav_rviz_plugins

#endif
