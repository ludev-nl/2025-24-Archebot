import math
import os
import sys
import time

import gpxpy
import gpxpy.gpx
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String, UInt8
from tf.transformations import euler_from_quaternion

from enums import Object


class Driver:
    def __init__(self):
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
        self.target_lat = 52.1648735
        self.target_long = 4.4645245
        self.MIN_DIST_FOR_HEADING = 0.5

        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.first_time = True

        self.follow_path = True
        self.join_at = 0
        self.rightdoor = 0

        rospy.Subscriber("/gps", NavSatFix, self.update_gps)
        rospy.Subscriber("/imu", Imu, self.update_imu)
        rospy.Subscriber("/object_detection", UInt8, self.update_object)
        rospy.Timer(rospy.Duration(0.1), self.drive)

        print("Driver initialized")

    def read_gpx_file(self):
        """
        Get the gps coordinates from the gpx file and make a list of those coordinates.
        """
        try:
            SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
            GPX_PATH = os.path.join(SCRIPT_DIR, "server/db/routes", "route.gpx")
            with open(GPX_PATH, "r") as gpx_file:
                gpx = gpxpy.parse(gpx_file)

            for point in gpx.tracks[0].segments[0].points:
                print("Point at ({0},{1})".format(point.latitude, point.longitude))
                self.coordinate_list.append((point.latitude, point.longitude))

            self.update_gps_target()

        except:
            print(f"File not found: {GPX_PATH}")
            sys.exit(1)

    def update_gps_target(self):
        """
        update target coordinates and remove first coordinate_list entry
        """
        self.target_lat, self.target_long = self.coordinate_list.pop(0)
        # print("NEW TARGET: ")
        # print(self.target_lat, self.target_long)

    def update_object(self, data):
        self.object = Object(data.data)

    def update_gps(self, data):
        self.lat = data.latitude
        self.long = data.longitude

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
        lat_to_m = 111320
        long_to_m = 40075000 * math.cos(math.radians(lat1)) / 360
        dx = (lon2 - lon1) * long_to_m
        dy = (lat2 - lat1) * lat_to_m
        distance = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        return distance, heading

    def gps_heading(self, lat1, lon1, lat2, lon2):
        delta_lon = math.radians(lon2 - lon1)
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)

        x = math.sin(delta_lon) * math.cos(lat2_rad)
        y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(
            lat2_rad
        ) * math.cos(delta_lon)

        heading = math.atan2(x, y)
        return heading

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
        if abs(heading_error) < 0.5:
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
        distance_to_target, target_heading = self.compute_distance_and_heading(
            self.lat, self.long, self.target_lat, self.target_long
        )
        if time.time() < self.next_heading_update:
            return distance_to_target, target_heading

        print("UPDATING HEADING")

        self.next_heading_update = time.time() + 2

        gps_yaw_valid = False
        if self.previous_lat is not None:
            dist_moved, _ = self.compute_distance_and_heading(
                self.previous_lat, self.previous_long, self.lat, self.long
            )
            if dist_moved > self.MIN_DIST_FOR_HEADING:
                self.gps_heading_est = self.gps_heading(
                    self.previous_lat, self.previous_long, self.lat, self.long
                )
                gps_yaw_valid = True

        self.previous_lat = self.lat
        self.previous_long = self.long

        if gps_yaw_valid:
            self.current_heading = self.apply_kalman_filter(
                self.gps_heading_est, self.imu_yaw
            )
        else:
            self.current_heading = self.imu_yaw

        return distance_to_target, target_heading

    def drive(self, _):
        if os.environ.get("archebot_start") != "true":
            return
        if self.first_time:
            self.read_gpx_file()
            self.first_time = False

        twist = Twist()

        now = time.time()
        dt = now - self.last_time if self.last_time else 0.1
        self.last_time = now

        distance_to_target, target_heading = self.update_heading()

        heading_error = math.atan2(
            math.sin(target_heading - self.current_heading),
            math.cos(target_heading - self.current_heading),
        )

        linear_x, angular_z = self.pid_control(distance_to_target, heading_error, dt)

        if distance_to_target < 0.5:
            # print("TARGET REACHED")
            linear_x = 0.0
            angular_z = 0.0
            self.update_gps_target()

        twist.linear.x = linear_x
        twist.angular.z = angular_z

        rospy.loginfo(
            f"[DRIVER] Dist: {distance_to_target:.2f}m | Heading Err: {math.degrees(heading_error):.2f} | Lin: {linear_x:.2f} | Ang: {angular_z:.2f}"
        )

        # check if object too close to robot in front
        if self.follow_path:
            if self.object == Object.LEFT:
                twist.linear.x = 0.0
                twist.angular.z = -0.4
                self.leave_path()
            elif self.object == Object.RIGHT:
                twist.linear.x = 0.0
                twist.angular.z = 0.4
                self.leave_path()
            else:
                pass
        else:
            if self.object == Object.NOBJECT:
                twist.linear.x = 0.4
                twist.angular.z = 0
                if time.time() > self.join_at:
                    self.follow_path = True
                self.rightdoor = 0
            elif self.rightdoor > 2:
                twist.linear.x = 0.0
                twist.angular.x = -0.4
                self.leave_path()
            elif self.object == Object.LEFT:
                if self.prev_object == Object.RIGHT:
                    self.rightdoor += 1
                twist.linear.x = 0.0
                twist.angular.z = -0.4
                self.leave_path()
            # elif self.object == Object.RIGHT:
            else:
                if self.prev_object == Object.LEFT:
                    self.rightdoor += 1
                twist.linear.x = 0.0
                twist.angular.z = 0.4
                self.leave_path()

        print(self.object)

        self.prev_object = self.object

        self.cmd_pub.publish(twist)

    def leave_path(self) -> None:
        self.follow_path = False
        self.join_at = time.time() + 5
