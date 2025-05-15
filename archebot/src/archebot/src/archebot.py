#!/usr/bin/env python3
#coding=utf-8

from sensor_msgs.msg import Image, NavSatFix, Imu
# from leo_msgs.msg import Imu
import sys, rospy, drive

def ros_exit() -> None:
    sys.exit()

if "__main__" == __name__:
    rospy.init_node("archebot", anonymous=True, log_level=rospy.INFO, disable_signals=False)
    rospy.on_shutdown(ros_exit)

    rospy.Subscriber("/imu/data", Imu, drive.imu_callback)
    rospy.Subscriber("/ublox_gps_node/fix", NavSatFix, drive.drive_towards_target)
    
    rospy.spin()
