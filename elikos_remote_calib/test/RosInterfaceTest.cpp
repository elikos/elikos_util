#define ROS_INTERFACE_TEST
#include "../src/core/RosInterface.h"
#include "RosInterfaceTest.h"

RosInterface::RosInterface()
{
    AllFunctions* dat =  new AllFunctions();
    internalData_ = static_cast<void*>(dat);
}

RosInterface::~RosInterface(){
    delete static_cast<AllFunctions*>(internalData_);
}

void RosInterface::spinOnce(){
    AllFunctions* callbacks = getRosInterfaceCallbacks(this);
    if(callbacks->spinOnce != nullptr){
        callbacks->spinOnce();
    }
}

RosInterface* RosInterface::get(int argc, char** argv, const char* nodeName){
    static RosInterface ri;
    return &ri;
}

AllFunctions* getRosInterfaceCallbacks(RosInterface* ri){
    return static_cast<AllFunctions*>(ri->getInternalData());
}