#include "ike_nav_rviz_plugins/ike_nav_panel.hpp"

#include "ui_ike_nav.h"

namespace ike_nav_rviz_plugins
{

IkeNavPanel::IkeNavPanel(QWidget * parent)
: rviz_common::Panel(parent), ui_(std::make_shared<Ui::IkeNavPanel>())
{
  ui_->setupUi(this);
}

IkeNavPanel::~IkeNavPanel() {}

}  // namespace ike_nav_rviz_plugins
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ike_nav_rviz_plugins::IkeNavPanel, rviz_common::Panel)