#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <std_srvs/srv/trigger.hpp>

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
  void initServiceClient();

  void addLogo();

  rclcpp::Node::SharedPtr createNewNode(const std::string & node_name);

  void onStartButtonClicked();

private:
  std::shared_ptr<Ui::IkeNavPanel> ui_;

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_waypoint_follower_client_;
};
}  // namespace ike_nav_rviz_plugins