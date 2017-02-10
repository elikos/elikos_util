/**
 * Classe coeur, qui s'occupe de gérer les paramètres de la calibration.
 */
#ifndef RM_CORE_H
#define RM_CORE_H

#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <elikos_ros/CalibPreprocessing.h>

class Core{
public:
    Core();
    void update();

    void sendCalibrationData(int gaussianKernelSize, int gaussianRepetitions);

private:
    ros::NodeHandle nodeHandle_;
    ros::Publisher publisher_;
};

#endif
