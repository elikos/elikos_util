#include "../src/core/Core.h"
#include "RosInterfaceTest.h"

#include <gtest/gtest.h>

TEST(CoreTests, update)
{
    RosInterfaceTest interface;
    bool wasSpinOnceCalled = false;
    interface.spinOnceCallback_ = [&wasSpinOnceCalled](){wasSpinOnceCalled = true;};

    Core core(&interface);
    core.update();
    ASSERT_TRUE(wasSpinOnceCalled);
}



int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
