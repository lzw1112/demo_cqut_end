#include "cqut_rviz_plugin/gear_display.hpp"

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

GearDisplay::GearDisplay() : current_gear_(0)
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
}

void GearDisplay::updateGearData(const cqut_msg::msg::Cmd::ConstSharedPtr & msg)
{
  current_gear_ = msg->delta * M_PI / 3.14;  // Assuming msg->report contains the gear information
}

}  // namespace cqut_overlay_rviz_plugin
