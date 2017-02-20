/**
 * Classe coeur, qui s'occupe de gérer les paramètres de la calibration.
 */
#ifndef RM_CORE_H
#define RM_CORE_H

#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <elikos_ros/CalibPreprocessing.h>
#include <vector>
#include <utility>

enum NodeType
{
    DETECTION
};

class Core{
public:
    Core();
    void update();

    void sendCalibrationData(int gaussianKernelSize, int gaussianRepetitions);
    //Retrouve la liste des vecteurs calibrables
    void getCalibratableNodes(std::vector<std::string>& calibratableNodes);
    //Rafraichit la liste des noeuds calibrables
    void refreshCalibratableNodes();

private:
    ros::NodeHandle nodeHandle_;
    ros::Publisher publisher_;

    std::vector<std::pair<NodeType, std::string>> calibratableNodes_;


};

#endif
