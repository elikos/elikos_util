#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped
import tf
import roslib
   
class elikos_vicon_remapping(object):

    def vicon_remapping(self):
        self.pose_actuelle = PoseStamped()

        vicon_ref_frame_id = rospy.get_param('/elikos_vicon_remapping/vicon_ref_frame_id')
        vicon_child_frame_id = rospy.get_param('/elikos_vicon_remapping/vicon_child_frame_id')
        elikos_ref_frame_id = rospy.get_param('/elikos_vicon_remapping/elikos_ref_frame_id')
        elikos_child_frame_id = rospy.get_param('/elikos_vicon_remapping/elikos_child_frame_id')

        listener = tf.TransformListener()
        broadcaster = tf.TransformBroadcaster()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform(vicon_ref_frame_id, vicon_child_frame_id, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            broadcaster.sendTransform(trans,
                        rot,
                        rospy.Time.now(),
                        elikos_child_frame_id,
                        elikos_ref_frame_id)    
            rate.sleep()

    def __init__(self):
        rospy.init_node('elikos_vicon_remapping', anonymous=True)
        self.vicon_remapping()

if __name__ == '__main__':
    try:
        remapper = elikos_vicon_remapping()
    except rospy.ROSInterruptException:
        pass