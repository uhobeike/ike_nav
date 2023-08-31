#include <rviz_common/panel.hpp>

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

private:
protected:
  std::shared_ptr<Ui::IkeNavPanel> ui_;
};
}  // namespace ike_nav_rviz_plugins