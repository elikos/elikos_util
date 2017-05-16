#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped
   
class elikos_vicon_remapping(object):

    def msg_callback(self, data):
        self.pose_actuelle = data
        self.pub.publish(self.pose_actuelle)
        print 'lol'

    def vicon_remapping(self):
        self.pose_actuelle = PoseStamped()

        in_topic = rospy.get_param('/elikos_vicon_remapping/in_topic')
        out_topic = rospy.get_param('/elikos_vicon_remapping/out_topic')
        self.pub = rospy.Publisher(out_topic, PoseStamped, queue_size=10)
        rospy.Subscriber(in_topic, PoseStamped, self.msg_callback)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

    def __init__(self):
        rospy.init_node('elikos_vicon_remapping', anonymous=True)
        self.vicon_remapping()

if __name__ == '__main__':
    try:
        remapper = elikos_vicon_remapping()
    except rospy.ROSInterruptException:
        pass