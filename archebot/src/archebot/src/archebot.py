#!/usr/bin/env python3
# coding=utf-8

import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, NavSatFix, Imu
from geometry_msgs.msg import Twist

from gps_logger import log_location
from object_avoidance import Avoider
from drive import Driver  # Assumes integrated Driver class with GPS/IMU logic

def ros_exit():
    rospy.loginfo("Shutting down ArcheBot node...")
    sys.exit(0)

def main():
    rospy.init_node("archebot", anonymous=True, log_level=rospy.INFO)
    rospy.on_shutdown(ros_exit)

    # Instantiate components
    avoider = Avoider()
    driver = Driver()

    # Subscriptions
    rospy.Subscriber("/d455_camera/depth/image_rect_raw", Image, avoider.initialize)
    rospy.Subscriber("/d455_camera/depth/image_rect_raw", Image, avoider.detect_object)
    rospy.Subscriber("/d455_camera/depth/image_rect_raw", Image, driver.drive)
    
    rospy.Subscriber("object_detection", String, driver.update_object)
    rospy.Subscriber("/ublox_gps_node/fix", NavSatFix, driver.update_gps)
    rospy.Subscriber("/imu/data", Imu, driver.update_imu)

    # Start ROS loop
    rospy.spin()

if __name__ == "__main__":
    main()
