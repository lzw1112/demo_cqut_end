#pragma once

#include "cqut_rviz_plugin/overlay_utils.hpp"

#include <QImage>
#include <QString>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include <cqut_msg/msg/cmd.hpp>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

namespace cqut_overlay_rviz_plugin
{

class GearDisplay
{
public:
  GearDisplay();
  void updateGearData(const cqut_msg::msg::Cmd::ConstSharedPtr & msg);

private:
  int current_gear_;  // Internal variable to store current gear
  QColor gray = QColor(194, 194, 194);
};

}  // namespace autoware_overlay_rviz_plugin
