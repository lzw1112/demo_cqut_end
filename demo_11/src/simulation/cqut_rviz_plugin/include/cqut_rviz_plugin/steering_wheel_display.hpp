#include "cqut_rviz_plugin/overlay_utils.hpp"

#include <QImage>
#include <QString>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include "cqut_msg/msg/cmd.hpp"

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

namespace cqut_overlay_rviz_plugin
{

class SteeringWheelDisplay
{
public:
  SteeringWheelDisplay();
  void drawSteeringWheel(QPainter & painter, const QRectF & backgroundRect);
  void updateSteeringData(const cqut_msg::msg::Cmd::ConstSharedPtr & msg);

private:
  float steering_angle_ = 0.0f;
  QColor gray = QColor(194, 194, 194);

  QImage wheelImage;
  QImage scaledWheelImage;
  QImage coloredImage(const QImage & source, const QColor & color);
};

}  // namespace cqut_overlay_rviz_plugin
