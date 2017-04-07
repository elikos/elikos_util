#ifndef REMOTE_CALIB_RATIO_LAYOUTED_FRAME_H
#define REMOTE_CALIB_RATIO_LAYOUTED_FRAME_H
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// Code de rqt_layouted_frame
// https://github.com/ros-visualization/rqt_common_plugins/blob/master/rqt_image_view/src/rqt_image_view/ratio_layouted_frame.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#include <QFrame>
#include <QImage>
#include <QLayout>
#include <QLayoutItem>
#include <QMutex>
#include <QPainter>
#include <QRect>
#include <QSize>

namespace remote_calib {
namespace rqt_image_view {

/**
 * RatioLayoutedFrame is a layout containing a single frame with a fixed aspect ratio.
 * The default aspect ratio is 4:3.
 */
class RatioLayoutedFrame
  : public QFrame
{

  Q_OBJECT

public:

  RatioLayoutedFrame(QWidget* parent, Qt::WindowFlags flags = 0);

  virtual ~RatioLayoutedFrame();

  const QImage& getImage() const;

  QImage getImageCopy() const;

  void setImage(const QImage& image);

  QRect getAspectRatioCorrectPaintArea();

  void resizeToFitAspectRatio();

  void setInnerFrameMinimumSize(const QSize& size);

  void setInnerFrameMaximumSize(const QSize& size);

  void setInnerFrameFixedSize(const QSize& size);

signals:

  void delayed_update();

  void mouseLeft(int x, int y);

protected slots:

  void onSmoothImageChanged(bool checked);

protected:

  void setAspectRatio(unsigned short width, unsigned short height);

  void paintEvent(QPaintEvent* event);

private:

  static int greatestCommonDivisor(int a, int b);

  void mousePressEvent(QMouseEvent * mouseEvent);

  QSize aspect_ratio_;

  QImage qimage_;
  mutable QMutex qimage_mutex_;

  bool smoothImage_;
};

}}

#endif