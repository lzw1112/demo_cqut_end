#pragma once

#ifndef Q_MOC_RUN
#include "cqut_rviz_plugin/gear_display.hpp"
#include "cqut_rviz_plugin/overlay_utils.hpp"
#include "cqut_rviz_plugin/speed_display.hpp"
#include "cqut_rviz_plugin/res_display.hpp"
#include "cqut_rviz_plugin/steering_wheel_display.hpp"
#include "cqut_rviz_plugin/turn_signals_display.hpp"

#include <QImage>
#include <QString>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

#include <memory>
#include <mutex>
#endif

namespace cqut_overlay_rviz_plugin
{
class SignalDisplay : public rviz_common::Display
{
  Q_OBJECT
public:
  SignalDisplay();
  ~SignalDisplay() override;

protected:
  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;
  void onEnable() override;
  void onDisable() override;

private Q_SLOTS:
  void updateOverlaySize();
  void updateSmallOverlaySize();
  void updateOverlayPosition();
  void updateOverlayColor();
  void update_cmd();
  void update_res();
  void update_velocity();

private:
  std::mutex mutex_;
  cqut_overlay_rviz_plugin::OverlayObject::SharedPtr overlay_;
  rviz_common::properties::IntProperty * property_width_;
  rviz_common::properties::IntProperty * property_height_;
  rviz_common::properties::IntProperty * property_left_;
  rviz_common::properties::IntProperty * property_top_;
  rviz_common::properties::ColorProperty * property_signal_color_;
  std::unique_ptr<rviz_common::properties::RosTopicProperty> cmd_topic_peoperty_;
  std::unique_ptr<rviz_common::properties::RosTopicProperty> res_topic_peoperty_;
  std::unique_ptr<rviz_common::properties::RosTopicProperty> vel_topic_peoperty_;

  void drawHorizontalRoundedRectangle(QPainter & painter, const QRectF & backgroundRect);
  void drawVerticalRoundedRectangle(QPainter & painter, const QRectF & backgroundRect);
  void setupRosSubscriptions();

  std::unique_ptr<SteeringWheelDisplay> steering_wheel_display_;
  std::unique_ptr<GearDisplay> gear_display_;
  std::unique_ptr<SpeedDisplay> speed_display_;
  std::unique_ptr<TurnSignalsDisplay> turn_signals_display_;
  std::unique_ptr<ResDisplay> res_display_;

  rclcpp::Subscription<cqut_msg::msg::Cmd>::SharedPtr sub_cmd_;
  rclcpp::Subscription<cqut_msg::msg::ResState>::SharedPtr sub_res_;
  rclcpp::Subscription<cqut_msg::msg::State>::SharedPtr sub_vel_;

  std::mutex property_mutex_;

  void update_cmd_msg(const cqut_msg::msg::Cmd::ConstSharedPtr &msg);
  void update_vel_msg(const cqut_msg::msg::State::ConstSharedPtr &msg);
  void update_res_msg(const cqut_msg::msg::ResState::ConstSharedPtr &msg);
  void drawWidget(QImage & hud);
};
}  // namespace cqut_overlay_rviz_plugin
