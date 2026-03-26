#pragma once

#include "cqut_rviz_plugin/overlay_utils.hpp"

#include <QImage>
#include <QString>
#include <QPushButton>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include <cqut_msg/msg/res_state.hpp>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

namespace cqut_overlay_rviz_plugin
{

class ResDisplay
{
public:
  ResDisplay();
  void drawSteeringWheel(QPainter & painter, const QRectF & backgroundRect);
  void updateSpeedData(const cqut_msg::msg::ResState::ConstSharedPtr & msg);

private:
  int current_state_;  // 0: stop, 1: ready_go
  QImage wheelImage;
  QImage scaledWheelImage;
  QImage coloredImage(const QImage & source, const QColor & color);
};

}  // namespace cqut_overlay_rviz_plugin
