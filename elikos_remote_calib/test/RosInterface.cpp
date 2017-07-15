#include <ros/ros.h>

#include "RosInterface.h"

//Visible à l'intérieur de l'unité de traduction uniquement
namespace{
struct InternalData{
    ros::NodeHandle nodeHandle;
};
}


RosInterface::RosInterface()
{
    InternalData* dat =  new InternalData();
    internalData_ = static_cast<void*>(dat);
}

RosInterface::~RosInterface(){
    delete static_cast<InternalData*>(internalData_);
}

void RosInterface::spinOnce(){
    ros::spinOnce();
}

RosInterface* RosInterface::get(int argc, char** argv, const char* nodeName){
    ros::init(argc, argv, nodeName);
    static RosInterface interface;
    return &interface;
}



