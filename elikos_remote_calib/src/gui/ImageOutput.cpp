#include "elikos_remote_calib/gui/ImageOutput.h"

#include <ros/console.h>

#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

/******************************************************************************
* Constructeur. Utile pour Qt.
******************************************************************************/
ImageOutput::ImageOutput(QWidget* parent)
    : QWidget(parent)
    , nodeHandle_(nullptr)
    , paused_(false)
{
    QVBoxLayout* layout = new QVBoxLayout(this);
    this->setLayout(layout);
    imageFrame_ = new remote_calib::rqt_image_view::RatioLayoutedFrame(this);
    layout->addWidget(imageFrame_);

    connect(imageFrame_, SIGNAL(mouseLeft(int, int)), this, SLOT(inner_mouseLeft(int, int)));
}

/******************************************************************************
* Assigne le node handle a la classe par agrégation. Ne PAS supprimer le 
* handle.
*
* @param nodeHandle     [in] le node handle a assigner par agrégation.
******************************************************************************/
void ImageOutput::setNodeHandle(const ros::NodeHandle& nodeHandle)
{
    nodeHandle_ = &nodeHandle;
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
    if(nodeHandle_ == nullptr){
        ROS_WARN("[ImageOutput] Node handle was null. Call ");
        return;
    }
    subscriber_.shutdown();
    //image vide
    imageFrame_->setImage(QImage());

    if(topicName == ""){
        return;
    }

    image_transport::ImageTransport it(*nodeHandle_);
    subscriber_ = it.subscribe(topicName, 1, &ImageOutput::callbackImage, this, image_transport::TransportHints(topicTransport));
}

/******************************************************************************
* Pause ou joue le flux d'images.
*
* @param paused     [in] si le flux devrait être pausé
******************************************************************************/
void ImageOutput::setPaused(bool paused)
{
    paused_ = paused;
}

/******************************************************************************
* @return est-ce que l'image est pausé en ce moment?
******************************************************************************/
bool ImageOutput::isPaused(){
    return paused_;
}

/******************************************************************************
* Retourne un pixel aux coordonées normalisés (x,y).
*
* @param xNorm      [in] la position en x, normalisé entre (0,1)
* @param yNorm      [in] la position en y, normalisé entre (0,1)
* @return la couleur du pixel a la position donnée
******************************************************************************/
cv::Vec3b ImageOutput::getPixel(float xNorm, float yNorm)
{
    if (lastDisplayedFrame_.size().width == 0 || lastDisplayedFrame_.size().height == 0){
        return cv::Vec3b(0,0,0);
    }
    
    if(xNorm < 0) xNorm = 0;
    else if(xNorm > 1) xNorm = 1;
    if(xNorm < 0) xNorm = 0;
    else if(xNorm > 1) xNorm = 1;


    int xPos = xNorm * lastDisplayedFrame_.size().width;
    int yPos = yNorm * lastDisplayedFrame_.size().height;


    return lastDisplayedFrame_.at<cv::Vec3b>(yPos, xPos);
}

/******************************************************************************
* @return la valeur de la taille normalisée d'un pixel.
******************************************************************************/
float ImageOutput::getNormalizedPixelWidth(){
    return 1.f/lastDisplayedFrame_.size().width;
}

/******************************************************************************
* @return la valeur de la taille normalisée d'un pixel.
******************************************************************************/
float ImageOutput::getNormalizedPixelHeight(){
    return 1.f/lastDisplayedFrame_.size().height;
}

void ImageOutput::togglePaused(){
    setPaused(!isPaused());
}

/******************************************************************************
* Slot appelé lorsqu'un click survient sur le composant.
******************************************************************************/
void ImageOutput::inner_mouseLeft(int x, int y){
    QRect rect = imageFrame_->contentsRect();
    emit mouseLeft(x/(float)rect.width(), y/(float)rect.height());
}

/******************************************************************************
* Callback des images qui viennent du topic. Affiche les images.
******************************************************************************/
void ImageOutput::callbackImage(const sensor_msgs::Image::ConstPtr& msg)
{
    if(paused_){
        return;
    }
    lastDisplayedFrame_;
    try
    {
        // First let cv_bridge do its magic
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        lastDisplayedFrame_ = cv_ptr->image;
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
            lastDisplayedFrame_ = cv_ptr->image;
        } else if (msg->encoding == "8UC1") {
            // convert gray to rgb
            cv::cvtColor(cv_ptr->image, lastDisplayedFrame_, CV_GRAY2RGB);
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

    // image must be copied since it uses the lastDisplayedFrame_ for storage which is asynchronously overwritten in the next callback invocation
    QImage image(lastDisplayedFrame_.data, lastDisplayedFrame_.cols, lastDisplayedFrame_.rows, lastDisplayedFrame_.step[0], QImage::Format_RGB888);
    imageFrame_->setImage(image);
}




