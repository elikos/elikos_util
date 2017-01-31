/**
 * Implementation of the abstract class RosInterface.
 */
#ifndef ROS_INTERFACE_IMPL_H
#define ROS_INTERFACE_IMPL_H

#include "RosInterface.h"




class RosInterfaceImpl : RosInterface{
private:
    RosInterfaceImpl();
public:
    void spinOnce();
    static RosInterfaceImpl* get();
};

#endif