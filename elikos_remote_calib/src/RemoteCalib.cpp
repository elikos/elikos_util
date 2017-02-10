/**
 * Lance la calibration à distance.
 */
#include <iostream>
#include <string>
#include "core/Core.h"


static const std::string NODE_NAME = "remote_calib";

int main(int argc, char** argv){
    ros::init(argc, argv, NODE_NAME);
    Core core;
    std::string a;
    std::cin >> a;

    core.sendCalibrationData(33, 44);

}