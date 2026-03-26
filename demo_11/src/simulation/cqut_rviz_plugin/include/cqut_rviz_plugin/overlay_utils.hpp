#pragma once

#include <QColor>
#include <QImage>

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayContainer.h>
#include <Overlay/OgreOverlayElement.h>
#include <Overlay/OgreOverlayManager.h>
#include <Overlay/OgrePanelOverlayElement.h>

#include <memory>
#include <string>

namespace cqut_overlay_rviz_plugin
{
class OverlayObject;

class ScopedPixelBuffer
{
public:
  explicit ScopedPixelBuffer(Ogre::HardwarePixelBufferSharedPtr pixel_buffer);
  virtual ~ScopedPixelBuffer();
  virtual Ogre::HardwarePixelBufferSharedPtr getPixelBuffer();
  virtual QImage getQImage(unsigned int width, unsigned int height);
  virtual QImage getQImage(OverlayObject & overlay);
  virtual QImage getQImage(unsigned int width, unsigned int height, QColor & bg_color);
  virtual QImage getQImage(OverlayObject & overlay, QColor & bg_color);

protected:
  Ogre::HardwarePixelBufferSharedPtr pixel_buffer_;
};

enum class VerticalAlignment : uint8_t { CENTER, TOP, BOTTOM };

enum class HorizontalAlignment : uint8_t { LEFT, RIGHT, CENTER };

class OverlayObject
{
public:
  using SharedPtr = std::shared_ptr<OverlayObject>;

  explicit OverlayObject(const std::string & name);
  virtual ~OverlayObject();

  virtual std::string getName() const;
  virtual void hide();
  virtual void show();
  virtual bool isTextureReady() const;
  virtual void updateTextureSize(unsigned int width, unsigned int height);
  virtual ScopedPixelBuffer getBuffer();
  virtual void setPosition(
    double hor_dist, double ver_dist, HorizontalAlignment hor_alignment = HorizontalAlignment::LEFT,
    VerticalAlignment ver_alignment = VerticalAlignment::TOP);
  virtual void setDimensions(double width, double height);
  virtual bool isVisible() const;
  virtual unsigned int getTextureWidth() const;
  virtual unsigned int getTextureHeight() const;

protected:
  const std::string name_;
  Ogre::Overlay * overlay_;
  Ogre::PanelOverlayElement * panel_;
  Ogre::MaterialPtr panel_material_;
  Ogre::TexturePtr texture_;
};
}  // namespace cqut_overlay_rviz_plugin