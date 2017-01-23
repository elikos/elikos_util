#!/usr/bin/env python
# Shortens a bag to a managable 10 seconds version
# Useful to to test bag manipulation scripts/programs without waiting 30 minutes
# Usage :
# python BagShortener.py <path to input bag> <path to output bag>
# The output bag will be overwritten
import rosbag
import rospy
import sys

if len(sys.argv) != 3 or not sys.argv[1].endswith(".bag") or not sys.argv[2].endswith(".bag") :
    print("Usage : 'python BagShortener.py /input/file/path.bag ../output/file/path.bag");
    print(sys.argv);
    sys.exit(2);


print("opening bag now");

with rosbag.Bag(sys.argv[1]) as bag, \
     rosbag.Bag(sys.argv[2], 'w') as outbag :
    print("bag open");

    startTime = bag.get_start_time();
    print(startTime+1);
    for topic, msg, t in bag.read_messages(start_time = rospy.Time(startTime), end_time = rospy.Time(startTime+10)):
        outbag.write(topic, msg, t);

    print("Wrote all messages. Exiting now.");
