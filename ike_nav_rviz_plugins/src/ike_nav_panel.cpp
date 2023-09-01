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
  disableButton();

  initSubscription();
  initServiceClient();

  connect(ui_->start, &QPushButton::clicked, this, &IkeNavPanel::onStartButtonClicked);
  connect(ui_->stop, &QPushButton::clicked, this, &IkeNavPanel::onStopButtonClicked);
  connect(ui_->cancel, &QPushButton::clicked, this, &IkeNavPanel::onCancelButtonClicked);

  timer_id_ = startTimer(100);
}

IkeNavPanel::~IkeNavPanel() {}

void IkeNavPanel::initSubscription()
{
  navigation_feedback_sub_ =
    client_node_->create_subscription<NavigateToGoal::Impl::FeedbackMessage>(
      "navigate_to_goal/_action/feedback", 1,
      [this](const NavigateToGoal::Impl::FeedbackMessage msg) {
        this->ui_->waypoint_id_value_label->setText(
          std::to_string(msg.feedback.waypoint_id).c_str());
        std::string distance_remaining = std::to_string(msg.feedback.distance_remaining) + " [m]";
        this->ui_->distance_remaining_value_label->setText(distance_remaining.c_str());

        if (msg.feedback.navigation_status == 0) {
          this->ui_->navigation_status_value_label->setText("Planning");
          QPalette palette = this->ui_->navigation_status_value_label->palette();
          palette.setColor(QPalette::WindowText, Qt::blue);
          this->ui_->navigation_status_value_label->setPalette(palette);
        } else if (msg.feedback.navigation_status == 1) {
          this->ui_->navigation_status_value_label->setText("Goal Reached");
          QPalette palette = this->ui_->navigation_status_value_label->palette();
          palette.setColor(QPalette::WindowText, Qt::green);
          this->ui_->navigation_status_value_label->setPalette(palette);
        }
      });
}

void IkeNavPanel::initServiceClient()
{
  start_waypoint_follower_client_ =
    client_node_->create_client<std_srvs::srv::Trigger>("start_waypoint_follower");

  stop_waypoint_follower_client_ =
    client_node_->create_client<std_srvs::srv::Trigger>("stop_waypoint_follower");

  cancel_waypoint_follower_client_ =
    client_node_->create_client<std_srvs::srv::Trigger>("cancel_waypoint_follower");
}

void IkeNavPanel::disableButton()
{
  ui_->stop->setEnabled(false);
  ui_->cancel->setEnabled(false);
  ui_->waypoint_save->setEnabled(false);
  ui_->waypoint_load->setEnabled(false);
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
  ui_->start->setEnabled(false);

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

  ui_->stop->setEnabled(true);
  ui_->cancel->setEnabled(true);
}

void IkeNavPanel::onStopButtonClicked()
{
  ui_->stop->setEnabled(false);

  using namespace std::chrono_literals;

  while (!stop_waypoint_follower_client_->wait_for_service(1s)) {
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
    stop_waypoint_follower_client_->async_send_request(request, response_received_callback);

  ui_->start->setEnabled(true);
}

void IkeNavPanel::onCancelButtonClicked()
{
  ui_->cancel->setEnabled(false);

  using namespace std::chrono_literals;

  while (!cancel_waypoint_follower_client_->wait_for_service(1s)) {
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
    cancel_waypoint_follower_client_->async_send_request(request, response_received_callback);

  ui_->start->setEnabled(true);
}

void IkeNavPanel::timerEvent(QTimerEvent * event)
{
  if (event->timerId() == timer_id_) rclcpp::spin_some(client_node_);
}

}  // namespace ike_nav_rviz_plugins
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ike_nav_rviz_plugins::IkeNavPanel, rviz_common::Panel)