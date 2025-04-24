#!/usr/bin/env python3
#coding=utf-8


import sys, rospy

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Image

import gps, depth_camera
from gps_logger import log_location
from object_avoidance import Avoider

def ros_exit() -> None:
    sys.exit()


if "__main__" == __name__:
    avoider = Avoider()

    rospy.init_node("archebot", anonymous=True, log_level=rospy.INFO, disable_signals=False)
    rospy.on_shutdown(ros_exit)

    # rospy.Subscriber("/ublox/fix", NavSatFix, gps.main)
    # rospy.Subscriber("/d455_camera/depth/image_rect_raw", Image, depth_camera.main)
    # rospy.Subscriber("/camera/image_raw", Image, camera.main)
    
    # rospy.Subscriber("/ublox/fix", NavSatFix, log_location)

    sub = rospy.Subscriber("/d455_camera/depth/image_rect_raw", Image, avoider.initialize)
    
    rospy.spin()
