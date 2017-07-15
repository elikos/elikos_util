#define ROS_INTERFACE_TEST

#include <gtest/gtest.h>

#include "../src/core/Core.h"
#include "RosInterfaceTest.h"


TEST(CoreTests, update)
{
    RosInterface* rosInterface = RosInterface::get(0, nullptr, "");
    AllFunctions* rosInterfaceCallbacks = getRosInterfaceCallbacks(rosInterface);

    bool wasSpinOnceCalled = false;
    rosInterfaceCallbacks->spinOnce = [&wasSpinOnceCalled](){wasSpinOnceCalled = true;};

    Core core(rosInterface);
    core.update();
    ASSERT_TRUE(wasSpinOnceCalled);
}



int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
