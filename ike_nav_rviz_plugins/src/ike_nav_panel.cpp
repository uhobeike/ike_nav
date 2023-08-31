#include "ike_nav_rviz_plugins/ike_nav_panel.hpp"

#include "ui_ike_nav.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace ike_nav_rviz_plugins
{

IkeNavPanel::IkeNavPanel(QWidget * parent)
: rviz_common::Panel(parent), ui_(std::make_shared<Ui::IkeNavPanel>())
{
  ui_->setupUi(this);

  std::string ike_nav_logo_path =
    ament_index_cpp::get_package_share_directory("ike_nav_rviz_plugins") +
    "/media/ike_nav_logo.png";

  QPixmap pixmap(ike_nav_logo_path.c_str());
  ui_->image_label->setPixmap(pixmap);
  ui_->image_label->setScaledContents(true);
}

IkeNavPanel::~IkeNavPanel() {}

}  // namespace ike_nav_rviz_plugins
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ike_nav_rviz_plugins::IkeNavPanel, rviz_common::Panel)