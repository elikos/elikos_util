#include "Core.h"


Core::Core()
{
    publisher_ = nodeHandle_.advertise<elikos_ros::CalibPreprocessing>("remote_calib_color", 2);
}

void Core::update(){
    ros::spinOnce();
}

void Core::sendCalibrationData(int gaussianKernelSize, int gaussianRepetitions){
    elikos_ros::CalibPreprocessing msg;
    msg.gaussianKernelSize = gaussianKernelSize;
    msg.gaussianRepetitions = gaussianRepetitions;
    publisher_.publish(msg);
}