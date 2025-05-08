#!/usr/bin/env python3
#coding=utf-8

from sensor_msgs.msg import Image, NavSatFix
import sys, rospy, shard_detection

def ros_exit() -> None:
    sys.exit()

if "__main__" == __name__:
    rospy.init_node("archebot", anonymous=True, log_level=rospy.INFO, disable_signals=False)
    rospy.on_shutdown(ros_exit)

    rospy.Subscriber("/ublox/fix", NavSatFix, shard_detection.save_location)
    rospy.Subscriber("/camera/image_raw", Image, shard_detection.shard_detection)
    
    rospy.spin()
