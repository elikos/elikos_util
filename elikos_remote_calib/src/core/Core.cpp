#include "Core.h"

Core::Core(RosInterface* interfaceRos)
    : interfaceRos_(interfaceRos)
{

}

void Core::update(){
    interfaceRos_->spinOnce();
}