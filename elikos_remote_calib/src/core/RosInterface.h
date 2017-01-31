/**
 * Interface entre le programme et ros.
 */
#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H


class RosInterface {
public:
    /**
     * Spins ros once (processes messages, callbacks, etc...)
     */
    virtual void spinOnce() = 0;
};

#endif
