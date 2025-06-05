import collections
import math
import os
import sys
import time

import gpxpy
import gpxpy.gpx
import numpy as np
import rospy
from enums import Object
from geographiclib.geodesic import Geodesic
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String, UInt8
from tf.transformations import euler_from_quaternion


class Driver:
    def __init__(self):
        self.lat_deque = collections.deque(maxlen=5)
        self.long_deque = collections.deque(maxlen=5)
        self.lat = 0.0
        self.long = 0.0
        self.imu_yaw = 0.0
        self.object = Object.NOBJECT
        self.prev_object = Object.NOBJECT
        self.last_time = None
        self.previous_lat = None
        self.previous_long = None
        self.fused_yaw = 0.0
        self.gps_heading_est = None
        self.kalman_gain = 0.1

        # PID
        self.angle_integral = 0.0
        self.angle_prev_error = 0.0
        self.distance_integral = 0.0
        self.distance_prev_error = 0.0

        self.next_heading_update = 0

        self.coordinate_list = []
        self.target_lat = 52.164944
        self.target_long = 4.464528
        self.MIN_DIST_FOR_HEADING = 0.5
        self.MIN_DIST_FOR_TARGET = 1.5

        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)
        self.first_time = True

        self.rightdoor = 0
        self.heading_timer = -1
        self.is_updating_heading = False
        self.heading_error = 0
        self.target_yaw = 0

        # rospy.Subscriber("/gps", NavSatFix, self.update_gps)
        # rospy.Subscriber("/imu", Imu, self.update_imu)
        rospy.Subscriber("/object_detection", UInt8, self.update_object)
        rospy.Timer(rospy.Duration(0.1), self.drive)

        print("Driver initialized")

    def read_gpx_file(self, gpx_path):
        """
        Get the gps coordinates from the gpx file and make a list of those coordinates.
        """
        if len(self.coordinate_list) != 0:
            print("ALREADY INITIALIZED")
            return

        try:
            with open(gpx_path, "r") as gpx_file:
                gpx = gpxpy.parse(gpx_file)
            print("iets")
            for point in gpx.tracks[0].segments[0].points:
                print("Point at ({0},{1})".format(point.latitude, point.longitude))
                self.coordinate_list.append((point.latitude, point.longitude))

            # append first coordinate to the end of the list so the rover
            # returns to the start after exploring all gps targets
            self.coordinate_list.append(self.coordinate_list[0])
            print(self.coordinate_list)

            self.update_gps_target()

        except FileNotFoundError as _:
            print(f"File not found: {gpx_path}")
            sys.exit(1)

    def update_gps_target(self):
        """
        update target coordinates and remove first coordinate_list entry
        """
        if len(self.coordinate_list) == 0:
            print("byebye")
            sys.exit(0)
        self.target_lat, self.target_long = self.coordinate_list.pop(0)
        # print("NEW TARGET: ")
        # print(self.target_lat, self.target_long)

    def update_object(self, data):
        self.object = Object(data.data)

    def update_gps(self, data):
        # discard gps data if no signal
        if data.status.status == -1:
            print("NO SIGNAL")
            return
        # discard gps data if uncertainty is above 10 meters
        elif (
            max(
                abs(data.position_covariance[0] ** 0.5),
                abs(data.position_covariance[4] ** 0.5),
            )
            > 2
        ):
            print("TOO HIGH UNCERTAINTY")
            return

        self.lat_deque.append(data.latitude)
        self.long_deque.append(data.longitude)

        self.lat = sum(self.lat_deque) / len(self.lat_deque)
        self.long = sum(self.long_deque) / len(self.long_deque)

    def update_imu(self, msg):
        orientation_q = msg.orientation
        quaternion = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        (_, _, yaw) = euler_from_quaternion(quaternion)
        self.imu_yaw = yaw

    def compute_distance_and_heading(self, lat1, lon1, lat2, lon2):
        result = Geodesic.WGS84.Inverse(lat1, lon1, lat2, lon2)
        # distance in meters
        distance = result["s12"]
        # heading in radians
        heading = math.radians(result["azi1"])

        # heading offset 270 degrees
        heading += 0.5 * math.pi
        if heading > math.pi:
            heading -= 2 * math.pi
        elif heading < -math.pi:
            heading += 2 * math.pi

        return distance, heading

    def apply_kalman_filter(self, gps_head, imu_head):
        self.fused_yaw = (1 - self.kalman_gain) * imu_head + self.kalman_gain * gps_head
        return math.atan2(math.sin(self.fused_yaw), math.cos(self.fused_yaw))

    def pid_control(self, distance, heading_error, dt):
        Kp_ang, Ki_ang, Kd_ang = 1.2, 0.0, 0.1
        Kp_lin, Ki_lin, Kd_lin = 0.5, 0.0, 0.05

        # Angular PID
        self.angle_integral += heading_error * dt
        angle_derivative = (heading_error - self.angle_prev_error) / dt
        angular_z = (
            Kp_ang * heading_error
            + Ki_ang * self.angle_integral
            + Kd_ang * angle_derivative
        )
        angular_z = max(-1.0, min(1.0, angular_z))
        self.angle_prev_error = heading_error

        # Linear PID
        if abs(heading_error) < self.MIN_DIST_FOR_HEADING:
            self.distance_integral += distance * dt
            distance_derivative = (distance - self.distance_prev_error) / dt
            linear_x = (
                Kp_lin * distance
                + Ki_lin * self.distance_integral
                + Kd_lin * distance_derivative
            )
            linear_x = max(0.0, min(0.4, linear_x))
            self.distance_prev_error = distance
        else:
            linear_x = 0.0

        return linear_x, angular_z

    def update_heading(self):
        print("UPDATING HEADING")

        self.next_heading_update = time.time() + 2

        gps_yaw_valid = False
        if self.previous_lat is not None:
            dist_moved, gps_head = self.compute_distance_and_heading(
                self.previous_lat, self.previous_long, self.lat, self.long
            )
            if dist_moved > self.MIN_DIST_FOR_HEADING:
                self.gps_heading_est = gps_head
                gps_yaw_valid = True

        self.previous_lat = self.lat
        self.previous_long = self.long

        if gps_yaw_valid:
            self.current_heading = self.apply_kalman_filter(
                self.gps_heading_est, self.imu_yaw
            )
        else:
            self.current_heading = self.imu_yaw

        # heading_error = math.atan2(
        #     math.sin(self.target_heading - self.current_heading),
        #     math.cos(self.target_heading - self.current_heading),
        # )  # hier

        heading_error = self.target_heading - self.current_heading

        if heading_error > math.pi:
            heading_error -= 2 * math.pi
        elif heading_error < -math.pi:
            heading_error += 2 * math.pi

        rospy.loginfo(
            f"[DRIVER] Dist: {self.distance_to_target:.2f}m | Heading Err: {math.degrees(heading_error):.2f}"
        )

        self.target_yaw = self.imu_yaw + heading_error
        if self.target_yaw > math.pi:
            self.target_yaw -= 2 * math.pi
        elif self.target_yaw < -math.pi:
            self.target_yaw += 2 * math.pi

        return heading_error

        # rospy.loginfo(
        #     f"[DRIVER] Dist: {distance_to_target:.2f}m | Heading Err: {math.degrees(heading_error):.2f} | Lin: {linear_x:.2f} | Ang: {angular_z:.2f}"
        # )

        # linear_x, angular_z = self.pid_control(distance_to_target, heading_error, dt)

    def avoid(self, twist):
        """
        check if object too close to robot in front and updates twist accordingly
        """
        # if self.object == Object.NOBJECT:
        #     twist.linear.x = 0.4
        #     # twist.linear.x = 0  # hier
        #     twist.angular.z = 0
        #     self.rightdoor = 0
        #     return False
        # elif self.rightdoor > 2:
        #     twist.linear.x = 0.0
        #     twist.angular.z = -0.4
        # elif self.object == Object.LEFT:
        #     if self.prev_object == Object.RIGHT:
        #         self.rightdoor += 1
        #     twist.linear.x = 0.0
        #     twist.angular.z = -0.4
        # # elif self.object == Object.RIGHT:
        # else:
        #     if self.prev_object == Object.LEFT:
        #         self.rightdoor += 1
        #     twist.linear.x = 0.0
        #     twist.angular.z = 0.4

        twist.linear.x = 0.4

        # return True
        return False

    def start_heading_timer(self) -> None:
        self.previous_long = self.long
        self.previous_lat = self.lat
        self.heading_timer = 2

    def stop_heading_timer(self) -> None:
        self.heading_timer = -1

    def update_heading_timer(self, dt) -> None:
        if self.heading_timer == -1:
            return

        self.heading_timer = max(self.heading_timer - dt, 0)

    def rotate(self) -> float:
        # print("is rotating")
        # print(self.imu_yaw)
        # print(self.target_yaw)
        # print(self.target_yaw - self.imu_yaw)
        # print("ENDYAWS")

        if abs(self.imu_yaw - self.target_yaw) < (math.pi / 180 * 5):
            print("END ROTATE")
            self.is_updating_heading = False
            self.start_heading_timer()
            return 0

        diff = self.target_yaw - self.imu_yaw
        angular_z = (
            0.2 if ((diff + math.pi) % (2 * math.pi) - math.pi) > 0 else -0.2
        )  # hier

        return angular_z

    def drive(self, _):
        # print(self.target_lat)
        # print(self.target_long)
        # check archebot_start flag
        if os.environ.get("archebot_start") != "true":
            return
        if self.first_time:
            self.first_time = False
            print("DIT MOET MAAR EEN KEER")
            SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
            GPX_PATH = os.path.join(SCRIPT_DIR, "server/db/routes", "route.gpx")
            self.read_gpx_file(GPX_PATH)

        # update dt time
        now = time.time()
        dt = now - self.last_time if self.last_time else 0.1
        self.last_time = now

        # init twist
        twist = Twist()
        # keep updating heading until heading error becomes zero
        if self.is_updating_heading:
            # update rotation depending on the heading error
            angular_z = self.rotate()

            twist.angular.z = angular_z
            self.cmd_pub.publish(twist)
            return

        # update target heading and distance
        self.distance_to_target, self.target_heading = (
            self.compute_distance_and_heading(
                self.lat, self.long, self.target_lat, self.target_long
            )
        )

        # check if already at target
        if self.distance_to_target < self.MIN_DIST_FOR_TARGET:
            print("TARGET REACHED")
            self.update_gps_target()
            return

        self.update_heading_timer(dt)

        # check if object was avoided
        avoided = self.avoid(twist)
        # print(self.object)
        self.prev_object = self.object

        # if object was avoided return early
        if avoided:
            self.stop_heading_timer()
            self.cmd_pub.publish(twist)
            return

        # if heading timer is off start the timer
        if self.heading_timer == -1:
            print("start timer")
            self.start_heading_timer()
            self.cmd_pub.publish(twist)
            return
        # don't update heading if timer is not zero
        elif self.heading_timer != 0:
            # print("TIMER:", self.heading_timer)
            self.cmd_pub.publish(twist)
            return

        # update rotation depending on the heading error
        self.heading_error = self.update_heading()

        # print("ERROR: ")
        # print(self.heading_error)
        # print("HEADING_CURR: ")
        # print(self.current_heading)

        if abs(self.heading_error) > 0.18:
            print("START ROTATE")
            self.stop_heading_timer()
            self.is_updating_heading = True
            return

        self.start_heading_timer()
        self.cmd_pub.publish(twist)
