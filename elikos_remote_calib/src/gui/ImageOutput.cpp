#include "elikos_remote_calib/gui/ImageOutput.h"

#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

/******************************************************************************
* Constructeur. Utile pour Qt.
******************************************************************************/
ImageOutput::ImageOutput(QWidget* parent)
    : QWidget(parent)
{
    QStackedLayout* layout = new QStackedLayout(this);
    this->setLayout(layout);
    imageFrame_ = new remote_calib::rqt_image_view::RatioLayoutedFrame(this);
    layout->addWidget(imageFrame_);
}

/******************************************************************************
* Change le topic duquel on affiche les images.
*
* @param topicName      [in] le nom du topic de base
* @param topicTransport [in] le type de transport de l'image (raw, compressed,
*                               etc...)
******************************************************************************/
void ImageOutput::setListenTopic(const std::string& topicName, const std::string& topicTransport)
{
    subscriber_.shutdown();
    //image vide
    imageFrame_->setImage(QImage());

    if(topicName == ""){
        return;
    }

    image_transport::ImageTransport it(nodeHandle_);
    subscriber_ = it.subscribe(topicName, 1, &ImageOutput::callbackImage, this, image_transport::TransportHints(topicTransport));
}

/******************************************************************************
* Callback des images qui viennent du topic. Affiche les images.
******************************************************************************/
void ImageOutput::callbackImage(const sensor_msgs::Image::ConstPtr& msg)
{
    static cv::Mat conversion_mat_;
    try
    {
        // First let cv_bridge do its magic
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        conversion_mat_ = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
        std::cerr << "test" << std::endl;
        try
        {
        // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
        if (msg->encoding == "CV_8UC3")
        {
            // assuming it is rgb
            conversion_mat_ = cv_ptr->image;
        } else if (msg->encoding == "8UC1") {
            // convert gray to rgb
            cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
        } else {
            qWarning("ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
            imageFrame_->setImage(QImage());
            return;
        }
        }
        catch (cv_bridge::Exception& e)
        {
        qWarning("ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
        imageFrame_->setImage(QImage());
        return;
        }
    }

    // image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
    QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0], QImage::Format_RGB888);
    imageFrame_->setImage(image);
}




