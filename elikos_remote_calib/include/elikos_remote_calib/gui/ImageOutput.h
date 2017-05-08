/******************************************************************************
* Classe qui permet de montrer facilement le contenu d'une image a l'écran.
******************************************************************************/
#ifndef REMOTE_CALIB_IMAGE_OUTPUT_H
#define REMOTE_CALIB_IMAGE_OUTPUT_H

#include <QWidget>
#include <QStackedLayout>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include "elikos_remote_calib/gui/RatioLayoutedFrameDup.h"


class ImageOutput : public QWidget
{
    Q_OBJECT
public:
    ImageOutput(QWidget* parent);
    void setNodeHandle(const ros::NodeHandle&);
    void setListenTopic(const std::string& topicName, const std::string& topicTransport);

    bool isPaused();
    void setPaused(bool paused);

    cv::Vec3b getPixel(float xNorm, float yNorm);
    float getNormalizedPixelWidth();
    float getNormalizedPixelHeight();

signals:
    void mouseLeft(float xNorm, float yNorm);
public slots:
    void togglePaused();
private slots:
    void inner_mouseLeft(int x, int y);
private:
    void callbackImage(const sensor_msgs::Image::ConstPtr& msg);

    remote_calib::rqt_image_view::RatioLayoutedFrame* imageFrame_;
    image_transport::Subscriber subscriber_;
    const ros::NodeHandle*  nodeHandle_;

    cv::Mat lastDisplayedFrame_;

    bool paused_;
};




#endif
