#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "ike_nav_msgs/action/navigate_to_goal.hpp"
#include <std_srvs/srv/trigger.hpp>

using NavigateToGoal = ike_nav_msgs::action::NavigateToGoal;

namespace Ui
{
class IkeNavPanel;
}

namespace ike_nav_rviz_plugins
{
class IkeNavPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit IkeNavPanel(QWidget * parent = nullptr);
  virtual ~IkeNavPanel();

protected:
  void initSubscription();
  void initServiceClient();

  void disableButton();
  void addLogo();

  rclcpp::Node::SharedPtr createNewNode(const std::string & node_name);

  void onStartButtonClicked();
  void onStopButtonClicked();
  void onCancelButtonClicked();

private:
  void timerEvent(QTimerEvent * event) override;

  std::shared_ptr<Ui::IkeNavPanel> ui_;

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Subscription<NavigateToGoal::Impl::FeedbackMessage>::SharedPtr navigation_feedback_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_waypoint_follower_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_waypoint_follower_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cancel_waypoint_follower_client_;

  int timer_id_;
};
}  // namespace ike_nav_rviz_plugins