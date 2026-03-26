#include "cqut_rviz_plugin/signal_display.hpp"

#include <QFontDatabase>
#include <QPainter>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_rendering/render_system.hpp>

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <mutex>
#include <string>

namespace cqut_overlay_rviz_plugin
{

SignalDisplay::SignalDisplay()
{
  property_width_ = new rviz_common::properties::IntProperty(
    "Width", 550, "Width of the overlay", this, SLOT(updateOverlaySize()));
  property_height_ = new rviz_common::properties::IntProperty(
    "Height", 100, "Height of the overlay", this, SLOT(updateOverlaySize()));
  property_left_ = new rviz_common::properties::IntProperty(
    "Left", 0, "Left position of the overlay", this, SLOT(updateOverlayPosition()));
  property_top_ = new rviz_common::properties::IntProperty(
    "Top", 10, "Top position of the overlay", this, SLOT(updateOverlayPosition()));
  property_signal_color_ = new rviz_common::properties::ColorProperty(
    "Signal Color", QColor(QString("#00E678")), "Color of the signal arrows", this,
    SLOT(updateOverlayColor()));

  // Initialize the component displays
  steering_wheel_display_ = std::make_unique<SteeringWheelDisplay>();
  gear_display_ = std::make_unique<GearDisplay>();
  speed_display_ = std::make_unique<SpeedDisplay>();
  turn_signals_display_ = std::make_unique<TurnSignalsDisplay>();
  res_display_ = std::make_unique<ResDisplay>();
}

void SignalDisplay::onInitialize()
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  rviz_common::Display::onInitialize();
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
  static int count = 0;
  std::stringstream ss;
  ss << "SignalDisplayObject" << count++;
  overlay_.reset(new cqut_overlay_rviz_plugin::OverlayObject(ss.str()));
  overlay_->show();
  updateOverlaySize();
  updateOverlayPosition();

  auto rviz_ros_node = context_->getRosNodeAbstraction();

  cmd_topic_peoperty_ = std::make_unique<rviz_common::properties::RosTopicProperty>("Cmd Topic", "/cqut/cmd", "cqut_msg::msg::Cmd", "", this, SLOT(update_cmd()));
  cmd_topic_peoperty_->initialize(rviz_ros_node);

  res_topic_peoperty_ = std::make_unique<rviz_common::properties::RosTopicProperty>("Res Topic", "/cqut/res", "cqut_msg::msg::ResState", "", this, SLOT(update_res()));
  res_topic_peoperty_->initialize(rviz_ros_node);

  vel_topic_peoperty_ = std::make_unique<rviz_common::properties::RosTopicProperty>("Vel Topic", "/cqut/base_pose_ground_truth", "cqut_msg::msg::State", "", this, SLOT(update_velocity()));
  vel_topic_peoperty_->initialize(rviz_ros_node);
}

void SignalDisplay::setupRosSubscriptions()
{
  update_cmd();
  update_res();
  update_velocity();
}

SignalDisplay::~SignalDisplay()
{
  std::lock_guard<std::mutex> lock(property_mutex_);
  overlay_.reset();

  sub_cmd_.reset();
  sub_res_.reset();
  sub_vel_.reset();

  steering_wheel_display_.reset();
  gear_display_.reset();
  speed_display_.reset();
  turn_signals_display_.reset();
  cmd_topic_peoperty_.reset();
}

void SignalDisplay::update(float /* wall_dt */, float /* ros_dt */)
{
  if (!overlay_) {
    return;
  }
  cqut_overlay_rviz_plugin::ScopedPixelBuffer buffer = overlay_->getBuffer();
  QImage hud = buffer.getQImage(*overlay_);
  hud.fill(Qt::transparent);
  drawWidget(hud);
}

void SignalDisplay::onEnable()
{
  std::lock_guard<std::mutex> lock(property_mutex_);
  if (overlay_) {
    overlay_->show();
  }
  setupRosSubscriptions();
}

void SignalDisplay::onDisable()
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  sub_cmd_.reset();
  sub_res_.reset();
  sub_vel_.reset();

  if (overlay_) {
    overlay_->hide();
  }
}

void SignalDisplay::update_cmd_msg(const cqut_msg::msg::Cmd::ConstSharedPtr &msg)
{
    std::lock_guard<std::mutex> lock(property_mutex_);
  if(steering_wheel_display_)
  {
      steering_wheel_display_->updateSteeringData(msg);
      queueRender();
  }

  if(gear_display_)
  {
      gear_display_->updateGearData(msg);
      queueRender();
  }
}

void SignalDisplay::update_res_msg(const cqut_msg::msg::ResState::ConstSharedPtr &msg)
{
  std::lock_guard<std::mutex> lock(property_mutex_);
  if(res_display_)
  {
      res_display_->updateSpeedData(msg);
      queueRender();
  }
}

void SignalDisplay::update_vel_msg(const cqut_msg::msg::State::ConstSharedPtr &msg)
{
  std::lock_guard<std::mutex> lock(property_mutex_);
  if(speed_display_)
  {
      speed_display_->updateSpeedData(msg);
      queueRender();
  }
  if(turn_signals_display_)
  {
      turn_signals_display_->updateTurnSignalsData(msg);
      queueRender();
  }
}

void SignalDisplay::drawWidget(QImage & hud)
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  if (!overlay_->isVisible()) {
    return;
  }

  QPainter painter(&hud);
  painter.setRenderHint(QPainter::Antialiasing, true);

  QRectF backgroundRect(0, 0, 350, hud.height());
  drawHorizontalRoundedRectangle(painter, backgroundRect);


  if (steering_wheel_display_) {
    steering_wheel_display_->drawSteeringWheel(painter, backgroundRect);
  }

  if (speed_display_) {
    speed_display_->drawSpeedDisplay(painter, backgroundRect);
  }
  if (turn_signals_display_) {
    turn_signals_display_->drawArrows(painter, backgroundRect, property_signal_color_->getColor());
  }

  if(res_display_) {
    res_display_->drawSteeringWheel(painter, backgroundRect);
  }

  painter.end();
}

void SignalDisplay::drawHorizontalRoundedRectangle(
  QPainter & painter, const QRectF & backgroundRect)
{
  painter.setRenderHint(QPainter::Antialiasing, true);
  QColor colorFromHSV;
  colorFromHSV.setHsv(0, 0, 29);  // Hue, Saturation, Value
  colorFromHSV.setAlphaF(0.60);   // Transparency

  painter.setBrush(colorFromHSV);

  painter.setPen(Qt::NoPen);
  painter.drawRoundedRect(
    backgroundRect, backgroundRect.height() / 2, backgroundRect.height() / 2);  // Circular ends
}
void SignalDisplay::drawVerticalRoundedRectangle(QPainter & painter, const QRectF & backgroundRect)
{
  painter.setRenderHint(QPainter::Antialiasing, true);
  QColor colorFromHSV;
  colorFromHSV.setHsv(0, 0, 0);  // Hue, Saturation, Value
  colorFromHSV.setAlphaF(0.65);  // Transparency

  painter.setBrush(colorFromHSV);

  painter.setPen(Qt::NoPen);
  painter.drawRoundedRect(
    backgroundRect, backgroundRect.width() / 2, backgroundRect.width() / 2);  // Circular ends
}

void SignalDisplay::reset()
{
  rviz_common::Display::reset();
  overlay_->hide();
}

void SignalDisplay::updateOverlaySize()
{
  std::lock_guard<std::mutex> lock(mutex_);
  overlay_->updateTextureSize(property_width_->getInt(), property_height_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
  queueRender();
}

void SignalDisplay::updateOverlayPosition()
{
  std::lock_guard<std::mutex> lock(mutex_);
  overlay_->setPosition(
    property_left_->getInt(), property_top_->getInt(), HorizontalAlignment::CENTER,
    VerticalAlignment::TOP);
  queueRender();
}

void SignalDisplay::updateOverlayColor()
{
  std::lock_guard<std::mutex> lock(mutex_);
  queueRender();
}

void SignalDisplay::update_cmd()
{
    sub_cmd_.reset();
    auto rviz_ros_node = context_->getRosNodeAbstraction().lock();
    sub_cmd_ = rviz_ros_node->get_raw_node()->create_subscription<cqut_msg::msg::Cmd>(
        cmd_topic_peoperty_->getTopicStd(), 
        rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable(),
        [this](const cqut_msg::msg::Cmd::SharedPtr msg) {
          update_cmd_msg(msg);
        }
    );
}

void SignalDisplay::update_res()
{
    sub_res_.reset();
    auto rviz_ros_node = context_->getRosNodeAbstraction().lock();
    sub_res_ = rviz_ros_node->get_raw_node()->create_subscription<cqut_msg::msg::ResState>(
        res_topic_peoperty_->getTopicStd(), 
        rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable(),
        [this](const cqut_msg::msg::ResState::SharedPtr msg) {
          update_res_msg(msg);
        }
    );
}

void SignalDisplay::update_velocity()
{
    sub_vel_.reset();
    auto rviz_ros_node = context_->getRosNodeAbstraction().lock();
    sub_vel_ = rviz_ros_node->get_raw_node()->create_subscription<cqut_msg::msg::State>(
        vel_topic_peoperty_ ->getTopicStd(), 
        rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable(),
        [this](const cqut_msg::msg::State::SharedPtr msg) {
          update_vel_msg(msg);
        }
    );
}

}  // namespace cqut_overlay_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cqut_overlay_rviz_plugin::SignalDisplay, rviz_common::Display)
