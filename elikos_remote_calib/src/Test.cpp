/**
 * Lance la calibration à distance.
 */
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ros/master.h>


static const std::string NODE_NAME = "remote_calib";
static const std::string DETECTION_TOPIC_NAME = "detection";

namespace str{
    bool endsWith(const std::string& fullString, const std::string& end){
        if(fullString.length() >= end.length()){
            return ( 0 == fullString.compare(fullString.length() - end.length(), end.length(), end));
        }else{
            return false;
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, NODE_NAME);

    std::vector<std::string> nodeNames;
    ros::master::getNodes(nodeNames);

    for(std::string name : nodeNames){
        if(str::endsWith(name, DETECTION_TOPIC_NAME)){
            std::cout << name << std::endl;
        }
    }

    return 0;

}