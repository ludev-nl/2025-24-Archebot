#!/usr/bin/env python3
# coding=utf-8
import os
import sys
import threading

import rospy
import shard_detection
from drive import Driver
from geometry_msgs.msg import Twist
from gps_logger import log_location
from object_avoidance import Avoider
from sensor_msgs.msg import Image, NavSatFix, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt8

# Add /server/src to PYTHONPATH
sys.path.append(os.path.join(os.path.dirname(__file__), "server", "src"))
from routes import app


def start_flask():
    app.run(host="0.0.0.0", port=5000)

from gps_logger import log_location
from object_avoidance import Avoider
from drive import Driver  # Assumes integrated Driver class with GPS/IMU logic

def ros_exit():
    rospy.loginfo("Shutting down ArcheBot node...")
    sys.exit(0)

def main():
    rospy.init_node(
        "archebot", anonymous=True, log_level=rospy.INFO, disable_signals=False
    )
    rospy.on_shutdown(ros_exit)
    
    # Start the webserver
    flask_thread = threading.Thread(target=start_flask)
    flask_thread.daemon = True
    flask_thread.start()

    # start avoider and driver instance
    rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rospy.Publisher("object_detection", UInt8, queue_size=10)

    # Instantiate components
    avoider = Avoider()
    driver = Driver()

    # Subscriptions
    rospy.Subscriber("/d455_camera/depth/image_rect_raw", Image, avoider.initialize)
    rospy.Subscriber("/d455_camera/depth/image_rect_raw", Image, avoider.detect_object)
    rospy.Subscriber("/d455_camera/depth/image_rect_raw", Image, driver.drive)
    
    rospy.Subscriber("/camera/image_raw", Image, shard_detection.shard_detection)
    rospy.Subscriber("object_detection", UInt8, driver.update_object)
    rospy.Subscriber("/ublox/fix", NavSatFix, shard_detection.save_location)
    rospy.Subscriber("/ublox/fix", NavSatFix, log_location)
    rospy.Subscriber("/ublox_gps_node/fix", NavSatFix, driver.update_gps)
    rospy.Subscriber("/imu/data", Imu, driver.update_imu)

    rospy.spin()

if __name__ == "__main__":
    main()
