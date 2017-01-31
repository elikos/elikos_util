/**
 * Classe coeur, qui s'occupe de gérer les paramètres de la calibration.
 */
#ifndef RM_CORE_H
#define RM_CORE_H

#include "RosInterface.h"

class Core{
public:
    Core(RosInterface* interfaceRos);
    void update();

private:
    RosInterface* interfaceRos_;
};

#endif
