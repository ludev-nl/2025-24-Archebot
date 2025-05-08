#!/usr/bin/env python3
#coding=utf-8


import sys, rospy

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import gps
from gps_logger import log_location
from object_avoidance import Avoider
from drive import drive

def ros_exit() -> None:
    sys.exit()


if "__main__" == __name__:
    rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rospy.Publisher("object_detection", String, queue_size=10)
    avoider = Avoider()

    rospy.init_node("archebot", anonymous=True, log_level=rospy.INFO, disable_signals=False)
    rospy.on_shutdown(ros_exit)

    # rospy.Subscriber("/ublox/fix", NavSatFix, gps.main)
    # rospy.Subscriber("/d455_camera/depth/image_rect_raw", Image, depth_camera.main)
    # rospy.Subscriber("/camera/image_raw", Image, camera.main)
    
    # rospy.Subscriber("/ublox/fix", NavSatFix, log_location)

    rospy.Subscriber("/d455_camera/depth/image_rect_raw", Image, avoider.initialize)
    rospy.Subscriber("/d455_camera/depth/image_rect_raw", Image, avoider.detect_object)    
    rospy.Subscriber("object_detection", String, drive)

    rospy.spin()
