// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "ike_nav_rviz_plugins/ike_nav_panel.hpp"

#include "ui_ike_nav.h"

#include <QFileDialog>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>
#include <yaml-cpp/yaml.h>

#include <fstream>

namespace ike_nav_rviz_plugins
{

IkeNavPanel::IkeNavPanel(QWidget * parent)
: rviz_common::Panel(parent),
  ui_(std::make_shared<Ui::IkeNavPanel>()),
  client_node_(createNewNode("ike_nav_rviz_panel"))
{
  ui_->setupUi(this);
  addLogo();
  ui_->stop->setEnabled(false);
  ui_->cancel->setEnabled(false);

  initSubscription();
  initServiceClient();

  // clang-format off
  connect(ui_->waypoint_load, &QPushButton::clicked, this, &IkeNavPanel::onWaypointLoadButtonClicked);
  connect(ui_->waypoint_save, &QPushButton::clicked, this, &IkeNavPanel::onWaypointSaveButtonClicked);

  connect(ui_->start, &QPushButton::clicked, this, &IkeNavPanel::onStartButtonClicked);
  connect(ui_->stop, &QPushButton::clicked, this, &IkeNavPanel::onStopButtonClicked);
  connect(ui_->cancel, &QPushButton::clicked, this, &IkeNavPanel::onCancelButtonClicked);

  // connect(ui_->cancel, &QPushButton::clicked, this, &IkeNavPanel::onCancelButtonClicked);
  connect(ui_->delete_waypoint, &QPushButton::clicked, this, &IkeNavPanel::onDeleteWaypointButtonClicked);
  connect(ui_->delete_all_waypoints, &QPushButton::clicked, this, &IkeNavPanel::onDeleteAllWaypointsButtonClicked);
  // clang-format on

  timer_id_ = startTimer(100);
}

IkeNavPanel::~IkeNavPanel() {}

void IkeNavPanel::initSubscription()
{
  navigation_feedback_sub_ =
    client_node_->create_subscription<NavigateToGoal::Impl::FeedbackMessage>(
      "navigate_to_goal/_action/feedback", 1,
      [this](const NavigateToGoal::Impl::FeedbackMessage::ConstSharedPtr msg) {
        this->ui_->waypoint_id_value_label->setText(
          std::to_string(msg->feedback.waypoint_id).c_str());
        std::string distance_remaining = std::to_string(msg->feedback.distance_remaining) + " [m]";
        this->ui_->distance_remaining_value_label->setText(distance_remaining.c_str());

        if (msg->feedback.navigation_status == 0) {
          this->ui_->navigation_status_value_label->setText("Planning");
          QPalette palette = this->ui_->navigation_status_value_label->palette();
          palette.setColor(QPalette::WindowText, Qt::blue);
          this->ui_->navigation_status_value_label->setPalette(palette);
        } else if (msg->feedback.navigation_status == 1) {
          this->ui_->navigation_status_value_label->setText("Goal Reached");
          QPalette palette = this->ui_->navigation_status_value_label->palette();
          palette.setColor(QPalette::WindowText, Qt::green);
          this->ui_->navigation_status_value_label->setPalette(palette);
        }
      });

  navigation_cancel_sub_ =
    client_node_->create_subscription<NavigateToGoal::Impl::GoalStatusMessage>(
      "navigate_to_goal/_action/status", 1,
      [this](const NavigateToGoal::Impl::GoalStatusMessage::ConstSharedPtr msg) {
        if (msg->status_list.back().status == 5) {
          this->ui_->navigation_status_value_label->setText("Cancel");
          QPalette palette = this->ui_->navigation_status_value_label->palette();
          palette.setColor(QPalette::WindowText, Qt::red);
          this->ui_->navigation_status_value_label->setPalette(palette);
        }
      });
}

void IkeNavPanel::initServiceClient()
{
  // clang-format off
  load_waypoint_yaml_client_ =
    client_node_->create_client<ike_nav_msgs::srv::LoadWaypointYaml>("load_waypoint_yaml");

  start_waypoint_follower_client_ =
    client_node_->create_client<std_srvs::srv::Trigger>("start_waypoint_follower");

  stop_waypoint_follower_client_ =
    client_node_->create_client<std_srvs::srv::Trigger>("stop_waypoint_follower");

  cancel_waypoint_follower_client_ =
    client_node_->create_client<std_srvs::srv::Trigger>("cancel_waypoint_follower");

  delete_waypoint_client_ = 
    client_node_->create_client<std_srvs::srv::Trigger>("delete_waypoint");

  delete_all_waypoints_client_ =
    client_node_->create_client<std_srvs::srv::Trigger>("delete_all_waypoints");

  get_waypoints_msg_client_ =
    client_node_->create_client<ike_nav_msgs::srv::GetWaypointsMsg>("get_waypoints_msg");
  // clang-format on
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

void IkeNavPanel::onWaypointLoadButtonClicked()
{
  ui_->waypoint_load->setEnabled(false);

  std::string waypoint_yaml_path =
    ament_index_cpp::get_package_share_directory("ike_launch") + "/config";
  QString path = QFileDialog::getOpenFileName(
    this, tr("Open Waypoint Yaml"), waypoint_yaml_path.c_str(), tr("Yaml Files (*.yaml)"));

  if (!path.toStdString().empty()) {
    using namespace std::chrono_literals;

    while (!load_waypoint_yaml_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          client_node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(client_node_->get_logger(), "service not available, waiting again...");
    }
    auto request = std::make_shared<ike_nav_msgs::srv::LoadWaypointYaml_Request>();

    request->waypoint_yaml_path = path.toStdString();

    using ServiceResponseFuture = rclcpp::Client<ike_nav_msgs::srv::LoadWaypointYaml>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
      auto result = future.get();
    };
    auto future_result =
      load_waypoint_yaml_client_->async_send_request(request, response_received_callback);
  }

  ui_->waypoint_load->setEnabled(true);
}

void IkeNavPanel::onWaypointSaveButtonClicked()
{
  ui_->waypoint_save->setEnabled(false);

  std::string waypoint_yaml_path =
    ament_index_cpp::get_package_share_directory("ike_launch") + "/config";
  std::string default_filename = "waypoint.yaml";
  QString path = QFileDialog::getSaveFileName(
    this, tr("Save Waypoint Yaml"), (waypoint_yaml_path + "/" + default_filename).c_str(),
    tr("Yaml Files (*.yaml)"));

  if (!path.toStdString().empty()) {
    using namespace std::chrono_literals;

    while (!get_waypoints_msg_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          client_node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(client_node_->get_logger(), "service not available, waiting again...");
    }
    auto request = std::make_shared<ike_nav_msgs::srv::GetWaypointsMsg_Request>();

    using ServiceResponseFuture = rclcpp::Client<ike_nav_msgs::srv::GetWaypointsMsg>::SharedFuture;
    auto response_received_callback = [this, path](ServiceResponseFuture future) {
      auto result = future.get();

      if (result->waypoints.waypoints.size()) {
        writeWaypointYaml(result->waypoints, path.toStdString());
      }
    };
    auto future_result =
      get_waypoints_msg_client_->async_send_request(request, response_received_callback);
  }

  ui_->waypoint_save->setEnabled(true);
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
  ui_->stop->setEnabled(false);

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

void IkeNavPanel::onDeleteWaypointButtonClicked()
{
  ui_->delete_waypoint->setEnabled(false);

  using namespace std::chrono_literals;

  while (!delete_waypoint_client_->wait_for_service(1s)) {
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
    delete_waypoint_client_->async_send_request(request, response_received_callback);

  ui_->delete_waypoint->setEnabled(true);
}

void IkeNavPanel::onDeleteAllWaypointsButtonClicked()
{
  ui_->delete_all_waypoints->setEnabled(false);
  ui_->stop->setEnabled(false);
  ui_->cancel->setEnabled(false);

  using namespace std::chrono_literals;

  while (!delete_all_waypoints_client_->wait_for_service(1s)) {
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
    delete_all_waypoints_client_->async_send_request(request, response_received_callback);

  ui_->start->setEnabled(true);
  ui_->delete_all_waypoints->setEnabled(true);
}

void IkeNavPanel::writeWaypointYaml(
  const ike_nav_msgs::msg::Waypoints waypoints, const std::string file_path)
{
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "waypoints";
  out << YAML::Value << YAML::BeginSeq;

  for (const auto & waypoint : waypoints.waypoints) {
    out << YAML::BeginMap;

    out << YAML::Key << "id" << YAML::Value << waypoint.id;

    out << YAML::Key << "position" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "x" << YAML::Value << waypoint.pose.position.x;
    out << YAML::Key << "y" << YAML::Value << waypoint.pose.position.y;
    out << YAML::EndMap;

    out << YAML::Key << "euler_angle" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "z" << YAML::Value << tf2::getYaw(waypoint.pose.orientation);
    out << YAML::EndMap;

    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;

  std::ofstream fout(file_path);
  fout << out.c_str();
  fout.close();
}

rclcpp::Node::SharedPtr IkeNavPanel::createNewNode(const std::string & node_name)
{
  std::string node = "__node:=" + node_name;
  auto options = rclcpp::NodeOptions().arguments({"--ros-args", "--remap", node, "--"});
  return std::make_shared<rclcpp::Node>("_", options);
}

void IkeNavPanel::timerEvent(QTimerEvent * event)
{
  if (event->timerId() == timer_id_) rclcpp::spin_some(client_node_);
}

}  // namespace ike_nav_rviz_plugins
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ike_nav_rviz_plugins::IkeNavPanel, rviz_common::Panel)