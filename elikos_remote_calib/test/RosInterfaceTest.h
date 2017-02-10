/**
 * Implementation de l'interface de ros pour les tests.
 */
#ifndef ROS_INTERFACE_TEST_H
#define ROS_INTERFACE_TEST_H
#include <functional>
#include "../src/core/RosInterface.h"

struct AllFunctions{
    std::function<void()>spinOnce;
};

AllFunctions* getRosInterfaceCallbacks(RosInterface* ri);

#endif
