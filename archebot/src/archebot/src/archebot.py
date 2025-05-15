#!/usr/bin/env python3
#coding=utf-8

from sensor_msgs.msg import Image, NavSatFix
import sys
import rospy
import shard_detection
import os
import threading

# Add /server/src to PYTHONPATH
sys.path.append(os.path.join(os.path.dirname(__file__), "server", "src"))
from routes import app

def start_flask():
    app.run(host="0.0.0.0", port=5000)

def ros_exit() -> None:
    sys.exit()

if "__main__" == __name__:
    # Start the webserver
    flask_thread = threading.Thread(target=start_flask)
    flask_thread.daemon = True
    flask_thread.start()
    
    # Initialize ros node
    rospy.init_node("archebot", anonymous=True, log_level=rospy.INFO, disable_signals=False)
    rospy.on_shutdown(ros_exit)

    # Subscribe on topics
    rospy.Subscriber("/ublox/fix", NavSatFix, shard_detection.save_location)
    rospy.Subscriber("/camera/image_raw", Image, shard_detection.shard_detection)
    
    # Block untill the node is shutdown
    rospy.spin()
