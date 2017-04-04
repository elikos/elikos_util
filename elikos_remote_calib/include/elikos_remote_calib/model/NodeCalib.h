/*******************************************************************************
* Classe qui gère l'interface entre ROS et un noeud de calibration. Elle est
* utilisé par l'interface graphique afin de calibrer les noeuds ROS.
*******************************************************************************/
#ifndef RC_MODEL_NODE_CALIB_H
#define RC_MODEL_NODE_CALIB_H

#include <ros/ros.h>
#include <elikos_remote_calib_client/Calibrator.h>


template<typename Msg>
class NodeCalib
{
public:
    // Constructeur de la classe
    NodeCalib(std::string& nodeName);

    // Envoie un message de calibration au noeud
    void sendCalibrationMessage(const Msg& message);


    // Retourne le NodeHandle de ros
    ros::NodeHandle& getNodeHandle();

private:
    ros::NodeHandle nodeHandle_;
    ros::Publisher calibrationMessagePublisher_;

};





//==============================================================================
//==============================================================================
//                          TEMPLATE DEFINITION
//==============================================================================
//==============================================================================


/*******************************************************************************
* Constructeur de la classe NoceCalib. Se connecte automatiquement au noeud 
* fourni.
*
* @param nodeName   [in] le nom du noeud auquel on doit se connecter
*******************************************************************************/
template<typename Msg>
NodeCalib<Msg>::NodeCalib(std::string& nodeName)
    : nodeHandle_()
{
    calibrationMessagePublisher_ = nodeHandle_.advertise<Msg>( nodeName + '/' + remote_calib::REMOTE_CALIB_NAMESPCACE + '/' + remote_calib::MESSAGE_TOPIC_NAME, 10 );
}

/*******************************************************************************
* Envoie un message de calibration au drome
*
* @param message        [in] le message de calibration a envoyer
*******************************************************************************/
template<typename Msg>
void NodeCalib<Msg>::sendCalibrationMessage(const Msg& message)
{
    calibrationMessagePublisher_.publish(message);
}


/*******************************************************************************
* Doc...
*******************************************************************************/
template<typename Msg>
ros::NodeHandle& NodeCalib<Msg>::getNodeHandle()
{
    return nodeHandle_;
}

#endif