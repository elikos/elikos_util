/**
 * Implementation de l'interface test de ros pour les tests.
 */
#ifndef ROS_INTERFACE_TEST_H
#define ROS_INTERFACE_TEST_H
#include <functional>

#include "../src/core/RosInterface.h"


class RosInterfaceTest : public RosInterface{
public:
    void spinOnce();


    std::function<void()> spinOnceCallback_;
};


#endif
