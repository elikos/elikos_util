/******************************************************************************
* Classe qui est utilisé pour faire l'interface entre ROS et les classes qui 
* servent a calibrer les noeuds. Permet d'envoyer falilement des messages au
* noeud calibré.
******************************************************************************/
#ifndef ELIKOS_REMOTE_CALIB_CALIBRATION_MESSAGE_MANAGER_H
#define ELIKOS_REMOTE_CALIB_CALIBRATION_MESSAGE_MANAGER_H
#include <ros/ros.h>

#include <elikos_remote_calib_client/Calibrator.h>

template <typename Msg>
class CalibrationMessageManager {
public:
    CalibrationMessageManager(const std::string& nodeName);
    void sendCalibraitonMessage(Msg m);
private:
    ros::NodeHandle nodeHandle_;
    ros::Publisher publisher_;
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
******************************************************************************/
template <typename Msg>
CalibrationMessageManager<Msg>::CalibrationMessageManager(const std::string& nodeName)
    : nodeHandle_()
{
    publisher_ = nodeHandle_.advertise<Msg>(
        ros::names::append(
            nodeName,
            ros::names::append(
                remote_calib::REMOTE_CALIB_NAMESPCACE,
                remote_calib::MESSAGE_TOPIC_NAME
            )
        ),
        4
    );
    
}

/******************************************************************************
* Envoie un message de calibration au noeud calibré.
*
* @param m      [in] le message a envoyer
******************************************************************************/
template <typename Msg>
void CalibrationMessageManager<Msg>::sendCalibraitonMessage(Msg m)
{
    publisher_.publish(m);
}

#endif