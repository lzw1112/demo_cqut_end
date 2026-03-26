#include "cqut_rviz_plugin/overlay_utils.hpp"

#include <QImage>
#include <QString>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include "cqut_msg/msg/state.hpp"

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

namespace cqut_overlay_rviz_plugin
{

class SpeedDisplay
{
public:
  SpeedDisplay();
  void drawSpeedDisplay(QPainter & painter, const QRectF & backgroundRect);
  void updateSpeedData(const cqut_msg::msg::State::ConstSharedPtr & msg);

private:
  float current_speed_;  // Internal variable to store current speed
  QColor gray = QColor(194, 194, 194);
};

}  // namespace cqut_overlay_rviz_plugin
