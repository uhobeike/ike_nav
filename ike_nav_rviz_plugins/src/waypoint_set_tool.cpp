
#include "ike_nav_rviz_plugins/waypoint_set_tool.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_rendering/geometry.hpp"
#include "rviz_rendering/mesh_loader.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"

#include <Ogre.h>

#include <memory>

namespace ike_nav_rviz_plugins
{

WaypointSetTool::WaypointSetTool()
: moving_flag_node_(NULL), current_flag_property_(NULL), waypoint_id_(0)
{
}

WaypointSetTool::~WaypointSetTool()
{
  for (std::size_t i = 0; i < flag_nodes_.size(); i++) {
    scene_manager_->destroySceneNode(flag_nodes_[i]);
    scene_manager_->destroySceneNode(text_nodes_[i]);
  }
}

void WaypointSetTool::onInitialize()
{
  flag_resource_ = "package://ike_nav_rviz_plugins/media/ike_nav_goal_flag.dae";

  if (!rviz_rendering::loadMeshFromResource(flag_resource_)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("plant_flag_tool"), "WaypointSetTool: failed to load model resource '%s'.",
      flag_resource_.c_str());
    return;
  }

  moving_flag_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity * entity = scene_manager_->createEntity(flag_resource_);
  moving_flag_node_->attachObject(entity);
  moving_flag_node_->setVisible(false);

  moving_text_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  auto new_waypoint_text = std::make_shared<rviz_rendering::MovableText>(Ogre::String("none"));
  new_waypoint_text->setTextAlignment(
    rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_CENTER);
  moving_text_node_->attachObject(new_waypoint_text.get());
  moving_text_node_->setVisible(false);
  waypoint_text_.push_back(new_waypoint_text);
}

void WaypointSetTool::activate()
{
  if (moving_flag_node_) {
    moving_flag_node_->setVisible(true);

    current_flag_property_ =
      new rviz_common::properties::VectorProperty("Flag " + QString::number(flag_nodes_.size()));
    current_flag_property_->setReadOnly(true);
    getPropertyContainer()->addChild(current_flag_property_);
  }

  if (moving_text_node_) {
    moving_text_node_->setVisible(true);
    waypoint_text_.back()->setCaption(std::to_string(waypoint_id_));
    // current_flag_property_ =
    //   new rviz_common::properties::VectorProperty("Flag " + QString::number(flag_nodes_.size()));
    // current_flag_property_->setReadOnly(true);
    // getPropertyContainer()->addChild(current_flag_property_);
  }
}

void WaypointSetTool::deactivate()
{
  if (moving_flag_node_) {
    moving_flag_node_->setVisible(false);
    delete current_flag_property_;
    current_flag_property_ = NULL;
  }

  if (moving_text_node_) {
    moving_text_node_->setVisible(false);
    // delete current_flag_property_;
    // current_flag_property_ = NULL;
  }
}

int WaypointSetTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (!moving_flag_node_ || !moving_text_node_) {
    return Render;
  }
  auto projection_finder = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
  auto projection = projection_finder->getViewportPointProjectionOnXYPlane(
    event.panel->getRenderWindow(), event.x, event.y);
  Ogre::Vector3 intersection = projection.second;
  if (projection.first) {
    moving_flag_node_->setVisible(true);
    moving_text_node_->setVisible(true);
    moving_flag_node_->setPosition(intersection);
    moving_text_node_->setPosition(intersection);
    current_flag_property_->setVector(intersection);

    if (event.leftDown()) {
      makeFlag(intersection);
      makeText(intersection);
      current_flag_property_ = NULL;
      return Render | Finished;
    }
  } else {
    moving_flag_node_->setVisible(false);
    moving_text_node_->setVisible(false);
  }
  return Render;
}

void WaypointSetTool::makeFlag(const Ogre::Vector3 & position)
{
  Ogre::SceneNode * node = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity * entity = scene_manager_->createEntity(flag_resource_);
  node->attachObject(entity);
  node->setVisible(true);
  node->setPosition(position);
  flag_nodes_.push_back(node);
}

void WaypointSetTool::makeText(const Ogre::Vector3 & position)
{
  Ogre::SceneNode * node = scene_manager_->getRootSceneNode()->createChildSceneNode();
  auto new_waypoint_text = std::make_shared<rviz_rendering::MovableText>(Ogre::String("none"));
  new_waypoint_text->setTextAlignment(
    rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_CENTER);
  node->attachObject(new_waypoint_text.get());
  node->setVisible(true);
  node->setPosition(position);
  new_waypoint_text->setCaption(std::to_string(++waypoint_id_));
  waypoint_text_.push_back(new_waypoint_text);
  text_nodes_.push_back(node);
}

void WaypointSetTool::save(rviz_common::Config config) const
{
  config.mapSetValue("Class", getClassId());

  rviz_common::Config flags_config = config.mapMakeChild("Flags");

  rviz_common::properties::Property * container = getPropertyContainer();
  int num_children = container->numChildren();
  for (int i = 0; i < num_children; i++) {
    rviz_common::properties::Property * position_prop = container->childAt(i);
    rviz_common::Config flag_config = flags_config.listAppendNew();
    flag_config.mapSetValue("Name", position_prop->getName());
    position_prop->save(flag_config);
  }
}

}  // namespace ike_nav_rviz_plugins

PLUGINLIB_EXPORT_CLASS(ike_nav_rviz_plugins::WaypointSetTool, rviz_common::Tool)
