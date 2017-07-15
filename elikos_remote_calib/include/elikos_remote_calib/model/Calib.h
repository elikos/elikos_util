/*******************************************************************************
 * Classe coeur, qui s'occupe de gérer les paramètres de la calibration.
 ******************************************************************************/
#ifndef RM_CALIB_H
#define RM_CALIB_H

#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <vector>
#include <map>
#include <utility>


#include "elikos_remote_calib/Observer.h"

enum NodeType
{
    DETECTION = 1
};
static const std::map<std::string, NodeType> TOPIC_TYPE_TO_NODE_TYPE = {
    {"elikos_remote_calib_client/CalibDetection", DETECTION}
};


class Calib{
public:
    //Constructeur de core.
    Calib(Observer* observer);

    //Rafraichit ROS
    void refresh();

    //Retrouve la liste des vecteurs calibrables
    void getCalibratableNodes(std::vector<std::string>& calibratableNodes);
    //Rafraichit la liste des noeuds calibrables
    void refreshCalibratableNodes();
    //Retrouve le type d'un noeud calibrable avec son nom
    NodeType getCalibratableNodeType(const std::string& nodeName);

private:
    ros::NodeHandle nodeHandle_;


    std::map<std::string, NodeType> calibratableNodes_;

    Observer* observer_;
    
};

#endif
