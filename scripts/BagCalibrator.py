#!/usr/bin/env python
import rosbag
import yaml
import sys

def help() :
    print("Usage : 'python BagShortener.py ../input/file/path.bag ../output/file/path.bag /config/file/of/cam /cam_topic_name'");
    sys.exit(2);

if len(sys.argv) != 5 :
    help();

scriptName = sys.argv[0];
sourceBagName = sys.argv[1];
destBagName = sys.argv[2];
configFileName = sys.argv[3];
camTopicName = sys.argv[4]

if not sourceBagName.endswith(".bag") or not destBagName.endswith(".bag") or not configFileName.endswith(".yaml") :
    help();




print("opening bag now");

with rosbag.Bag(sourceBagName) as bag, \
     rosbag.Bag(destBagName, 'w') as outbag, \
     open(configFileName) as cfg_file:
    print("bag open");

    cfg = yaml.load(cfg_file);
    print("Configuration loaded");

    modMsgsCount = 0;
    targetTopic = camTopicName + "/camera_info";


    for topic, msg, t in bag.read_messages():
        if topic == targetTopic :
            msg.height = cfg['image_height'];
            msg.width = cfg['image_width'];

            msg.distortion_model = cfg['distortion_model'];
            msg.D = cfg['distortion_coefficients']['data'];
            msg.K = cfg['camera_matrix']['data'];
            msg.R = cfg['rectification_matrix']['data'];
            msg.P = cfg['projection_matrix']['data'];
            modMsgsCount += 1;
            
        outbag.write(topic, msg, t);

    print("Wrote all messages. Exiting now.");
    print("Calibrated " + str(modMsgsCount) + " to camera " + targetTopic);





