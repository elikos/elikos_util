#include <ros/ros.h>

#include "RosInterfaceImpl.h"



RosInterfaceImpl::RosInterfaceImpl(){
}

void RosInterfaceImpl::spinOnce(){
    ros::spinOnce();
}

RosInterfaceImpl* RosInterfaceImpl::get(){
    static RosInterfaceImpl interface;
    return &interface;
}

