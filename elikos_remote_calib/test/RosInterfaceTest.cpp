#include "RosInterfaceTest.h"


void RosInterfaceTest::spinOnce(){
    if(spinOnceCallback_ != nullptr){
        spinOnceCallback_();
    }
}


