/******************************************************************************
* Classe qui est utilisé pour faire l'interface entre ROS et les classes qui 
* servent a calibrer les noeuds. Permet d'envoyer falilement des messages au
* noeud calibré.
******************************************************************************/
#ifndef ELIKOS_REMOTE_CALIB_CALIBRATION_MESSAGE_MANAGER_H
#define ELIKOS_REMOTE_CALIB_CALIBRATION_MESSAGE_MANAGER_H
#include <ros/ros.h>

#include <elikos_remote_calib_client/Calibrator.h>

template <typename Msg, typename T>
class CalibrationMessageManager {
public:
    CalibrationMessageManager(const std::string& nodeName, void(T::*fp)(const boost::shared_ptr< Msg const > &), T* instance);
    void sendCalibraitonMessage(Msg m);
private:
    ros::NodeHandle nodeHandle_;
    ros::Publisher publisher_;
    ros::Subscriber subscriber_;
};

//==============================================================================
//==============================================================================
//                         Définition des templates
//==============================================================================
//==============================================================================


/******************************************************************************
* Constructeur de la classe. Initialise les messages.
*
* @param nodeName   [in] le nom du noeud auquel se connecter.
* @param fp         [in] le pointeur de la fonction a appeler
* @param instance   [in] l'instance sur laquelle appeler la fonction
******************************************************************************/
template <typename Msg, typename T>
CalibrationMessageManager<Msg, T>::CalibrationMessageManager(const std::string& nodeName, void(T::*fp)(const boost::shared_ptr< Msg const > &), T* instance)
    : nodeHandle_()
{
    std::string topicName = ros::names::append(
            nodeName,
            ros::names::append(
                remote_calib::REMOTE_CALIB_NAMESPCACE,
                remote_calib::MESSAGE_TOPIC_NAME
            )
        );
    publisher_ = nodeHandle_.advertise<Msg>(topicName, 4);
    subscriber_ = nodeHandle_.subscribe<Msg, T>(topicName, 4, fp, instance);
    
}

/******************************************************************************
* Envoie un message de calibration au noeud calibré.
*
* @param m      [in] le message a envoyer
******************************************************************************/
template <typename Msg, typename T>
void CalibrationMessageManager<Msg, T>::sendCalibraitonMessage(Msg m)
{
    publisher_.publish(m);
}

#endif