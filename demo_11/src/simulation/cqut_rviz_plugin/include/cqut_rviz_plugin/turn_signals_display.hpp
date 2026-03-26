#include "cqut_rviz_plugin/overlay_utils.hpp"

#include <QImage>
#include <QString>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include <cqut_msg/msg/state.hpp>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

#include <chrono>

namespace cqut_overlay_rviz_plugin
{

class TurnSignalsDisplay
{
public:
  TurnSignalsDisplay();
  void drawArrows(QPainter & painter, const QRectF & backgroundRect, const QColor & color);
  void updateTurnSignalsData(const cqut_msg::msg::State::ConstSharedPtr & msg);

private:
  QImage arrowImage;
  QColor gray = QColor(79, 79, 79);

  double current_turn_signal_;    // Internal variable to store turn signal state
  QImage coloredImage(const QImage & source, const QColor & color);

  std::chrono::steady_clock::time_point last_toggle_time_;
  bool blink_on_ = false;
  const std::chrono::milliseconds blink_interval_{500};  // Blink interval in milliseconds
};

}  // namespace cqut_overlay_rviz_plugin
