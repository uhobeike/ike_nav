#include "ike_nav_rviz_plugins/ike_nav_panel.hpp"

#include "ui_ike_nav.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace ike_nav_rviz_plugins
{

IkeNavPanel::IkeNavPanel(QWidget * parent)
: rviz_common::Panel(parent),
  ui_(std::make_shared<Ui::IkeNavPanel>()),
  client_node_(createNewNode("ike_nav_rviz_panel"))
{
  ui_->setupUi(this);
  addLogo();

  initServiceClient();

  connect(ui_->start, &QPushButton::clicked, this, &IkeNavPanel::onStartButtonClicked);
}

IkeNavPanel::~IkeNavPanel() {}

void IkeNavPanel::initServiceClient()
{
  start_waypoint_follower_client_ =
    client_node_->create_client<std_srvs::srv::Trigger>("start_waypoint_follower");
}

void IkeNavPanel::addLogo()
{
  std::string ike_nav_logo_path =
    ament_index_cpp::get_package_share_directory("ike_nav_rviz_plugins") +
    "/media/ike_nav_logo.png";

  QPixmap pixmap(ike_nav_logo_path.c_str());
  ui_->image_label->setPixmap(pixmap);
  ui_->image_label->setScaledContents(true);
}

rclcpp::Node::SharedPtr IkeNavPanel::createNewNode(const std::string & node_name)
{
  std::string node = "__node:=" + node_name;
  auto options = rclcpp::NodeOptions().arguments({"--ros-args", "--remap", node, "--"});
  return std::make_shared<rclcpp::Node>("_", options);
}

void IkeNavPanel::onStartButtonClicked()
{
  using namespace std::chrono_literals;

  while (!start_waypoint_follower_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        client_node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(client_node_->get_logger(), "service not available, waiting again...");
  }
  auto request = std::make_shared<std_srvs::srv::Trigger_Request>();

  using ServiceResponseFuture = rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
    auto result = future.get();
  };
  auto future_result =
    start_waypoint_follower_client_->async_send_request(request, response_received_callback);
}

}  // namespace ike_nav_rviz_plugins
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ike_nav_rviz_plugins::IkeNavPanel, rviz_common::Panel)