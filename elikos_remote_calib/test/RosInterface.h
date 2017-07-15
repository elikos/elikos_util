/**
 * Interface entre le programme et ros.
 */
#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include <string>

class RosInterface {
private:
    RosInterface();
public:
    ~RosInterface();
    /**
     * Spins ros once (processes messages, callbacks, etc...)
     */
    void spinOnce();

    /**
    * Declares that we will publish Mgs messages on topicName
    * @param Msg the type of message
    * @param topicName the name of the topic to use
    * @param messageBufferQueueSize the size of the message queue
    */
    template<typename Msg>
    void advertiseTopic(std::string topicName, int messageBufferQueueSize = 10);



    static RosInterface* get(int argc, char** argv, const char* nodeName);
    inline void* getInternalData(){return internalData_;}
private:
    void* internalData_;
};

#ifdef ROS_INTERFACE_TEST
#include "../../test/RosInterfaceTest.h"
#else
#include "RosInterfaceTemplate.h"
#endif

#endif
