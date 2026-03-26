#include "cqut_rviz_plugin/res_display.hpp"

#include <QFontDatabase>
#include <QPainter>
#include <ament_index_cpp/get_package_share_directory.hpp>
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
#include <string>

namespace cqut_overlay_rviz_plugin
{

ResDisplay::ResDisplay() : current_state_(0)
{
  std::string package_path =
    ament_index_cpp::get_package_share_directory("cqut_rviz_plugin");
  std::string font_path = package_path + "/assets/font/Quicksand/static/Quicksand-Regular.ttf";
  std::string font_path2 = package_path + "/assets/font/Quicksand/static/Quicksand-Bold.ttf";
  int fontId = QFontDatabase::addApplicationFont(
    font_path.c_str());  // returns -1 on failure (see docs for more info)
  int fontId2 = QFontDatabase::addApplicationFont(
    font_path2.c_str());  // returns -1 on failure (see docs for more info)
  if (fontId == -1 || fontId2 == -1) {
    std::cout << "Failed to load the Quicksand font.";
  }

  // Load the wheel image
  std::string image_path = package_path + "/assets/images/res.png";
  wheelImage.load(image_path.c_str());
  scaledWheelImage = wheelImage.scaled(54, 54, Qt::KeepAspectRatio, Qt::SmoothTransformation);
}

void ResDisplay::updateSpeedData(const cqut_msg::msg::ResState::ConstSharedPtr & msg)
{
    if(msg->emergency == false && msg->push_button == true)
        current_state_ = 1;
    else if(msg->emergency == true && msg->push_button == false)
        current_state_ = 0;
}

void ResDisplay::drawSteeringWheel(QPainter & painter, const QRectF & backgroundRect)
{
    // Enable Antialiasing for smoother drawing
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

    QImage wheel;
    if(current_state_ == 0)
    {
        wheel = coloredImage(scaledWheelImage, Qt::red);
    }
    else
        wheel = coloredImage(scaledWheelImage, Qt::green);

  // Calculate the position
  int wheelCenterX = backgroundRect.left() + wheel.width() + 240;
  int wheelCenterY = backgroundRect.height() / 2;

  QPointF drawPoint(
    wheelCenterX - wheel.width() / 2, wheelCenterY - wheel.height() / 2);

  // Draw the rotated image
  painter.drawImage(drawPoint.x(), drawPoint.y() - 5, wheel);

    QString steeringAngleStringAfterModulo;
    if(current_state_ == 0)
        steeringAngleStringAfterModulo = "Stop";
    else
        steeringAngleStringAfterModulo = "Ready";

  // Draw the steering angle text
  QFont steeringFont("Quicksand", 9, QFont::Bold);
  painter.setFont(steeringFont);
  if(current_state_ == 0)
    painter.setPen(Qt::red);
    else
        painter.setPen(Qt::green);
  
  QRect steeringRect(
    wheelCenterX - wheelImage.width() / 2, wheelCenterY - wheelImage.height() / 2 + 25,
    wheelImage.width(), wheelImage.height());
  painter.drawText(steeringRect, Qt::AlignCenter, steeringAngleStringAfterModulo);
}

QImage ResDisplay::coloredImage(const QImage & source, const QColor & color)
{
  QImage result = source;
  QPainter p(&result);
  p.setCompositionMode(QPainter::CompositionMode_SourceAtop);
  p.fillRect(result.rect(), color);
  p.end();
  return result;
}

}  // namespace cqut_overlay_rviz_plugin
