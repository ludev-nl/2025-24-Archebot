import collections
import math
import os
import sys
import time

import gpxpy
import rospy
from geographiclib.geodesic import Geodesic
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from tf.transformations import euler_from_quaternion

from enums import Object


class Driver:
    def __init__(self):
        # CONSTANTS:
        self.KALMAN_GAIN = 0.1  # used in kalman filter
        # minimum distance required to be driven to update heading based on GPS
        self.MIN_DIST_FOR_HEADING = 0.5
        # minimum distance required to be considered to have reached the target
        self.MIN_DIST_FOR_TARGET = 1.5
        # offset to make heading of 0 point north = 90 degrees
        self.HEADING_OFFSET = 0.5 * math.pi
        # minimum acceptable heading error = 10 degrees
        self.MIN_ACCEPTABLE_HEADING_ERR = 10 / 180 * math.pi

        # VARS
        # circular buffer to average the current latitude and longitude over 5 measurements
        self.lat_deque = collections.deque(maxlen=5)
        self.long_deque = collections.deque(maxlen=5)
        # current latitude and longitude
        self.lat = 0.0
        self.long = 0.0
        # current heading/bearing according to the IMU
        self.imu_yaw = 0.0

        # status of what the robot sees:
        #   NOBJECT: robot sees no object
        #   LEFT:    robot sees object to its left
        #   RIGHT:   robot sees object to its right
        self.object = Object.NOBJECT
        # previus state of self.object
        self.prev_object = Object.NOBJECT

        # previous recorded time when drive() was run
        self.last_time = None
        # used together with self.lat/long to calculate heading/bearing of robot based on GPS
        self.previous_lat = None
        self.previous_long = None

        # PID
        self.angle_integral = 0.0
        self.angle_prev_error = 0.0
        self.distance_integral = 0.0
        self.distance_prev_error = 0.0

        # list of target coordinates
        self.coordinate_list = []

        # used to publish twist and make rover drive
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)
        # prevents read_gpx() from running twice
        self.first_time = True

        # counts how many times rover switches between seeing an object left and right
        # while turning to avoid it.
        # when above 2 the rover will keep turning right to prevent it from getting
        # stuck avoiding two objects left and right.
        self.object_counter = 0
        # timer when to update heading using GPS
        self.heading_timer = -1
        self.is_updating_heading = False
        # the heading target for the IMU to reach
        # used in turning the rover in the right direction
        self.target_yaw = 0

        # update self.object using /object_detection
        rospy.Subscriber("/object_detection", UInt8, self.update_object)
        # run self.drive() every 0.1 seconds
        rospy.Timer(rospy.Duration(0.1), self.drive)

        print("Driver initialized")

    def read_gpx_file(self, gpx_path):
        """
        Get the gps coordinates from the gpx file and make a list of those coordinates.
        """
        # don't initialize again if already initialized
        if len(self.coordinate_list) != 0:
            return

        try:
            with open(gpx_path, "r") as gpx_file:
                gpx = gpxpy.parse(gpx_file)
            for point in gpx.tracks[0].segments[0].points:
                print("Point at ({0},{1})".format(point.latitude, point.longitude))
                self.coordinate_list.append((point.latitude, point.longitude))

            # append first coordinate to the end of the list so the rover
            # returns to the start after exploring all gps targets
            self.coordinate_list.append(self.coordinate_list[0])

            # set first coordinate as target
            self.update_gps_target()

        except FileNotFoundError as _:
            print(f"File not found: {gpx_path}", file=sys.stderr)
            sys.exit(1)

    def update_gps_target(self):
        """
        update target coordinates and remove first coordinate_list entry
        """
        # exit if there are no more targets to reach
        if len(self.coordinate_list) == 0:
            print("byebye")
            sys.exit(0)
        self.target_lat, self.target_long = self.coordinate_list.pop(0)

    def update_object(self, data):
        """
        updates the self.object variable to what Avoider in object_avoidance.py
        sees.

        gets data from /object_detection rostopic published in Avoider
        """
        self.object = Object(data.data)

    def update_gps(self, data):
        """
        updates the self.lat and self.long to the latitude and longitude that the
        GPS provides.
        """
        # discard gps data if no signal
        if data.status.status == -1:
            return
        # discard gps data if uncertainty is above 10 meters
        elif (
            max(
                abs(data.position_covariance[0] ** 0.5),
                abs(data.position_covariance[4] ** 0.5),
            )
            > 2
        ):
            return

        self.lat_deque.append(data.latitude)
        self.long_deque.append(data.longitude)

        # self.lat/long is average of self.lat_deque/long_deque
        self.lat = sum(self.lat_deque) / len(self.lat_deque)
        self.long = sum(self.long_deque) / len(self.long_deque)

    def update_imu(self, msg):
        """
        calculates the current heading using the data that the IMU provides
        """
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
        """
        compute the distance and heading between two points
        """
        # calculate the distance and heading using geographiclib
        result = Geodesic.WGS84.Inverse(lat1, lon1, lat2, lon2)
        # distance in meters
        distance = result["s12"]
        # heading in radians
        heading = math.radians(result["azi1"])

        # heading offset
        heading += self.HEADING_OFFSET
        # keep heading between -pi and +pi
        if heading > math.pi:
            heading -= 2 * math.pi
        elif heading < -math.pi:
            heading += 2 * math.pi

        return distance, heading

    def apply_kalman_filter(self, gps_head, imu_head):
        """
        apply kalman filter to gps and imu heading to better estimate the true heading
        """
        fused_yaw = (1 - self.KALMAN_GAIN) * imu_head + self.KALMAN_GAIN * gps_head
        return math.atan2(math.sin(fused_yaw), math.cos(fused_yaw))

    def pid_control(self, distance, heading_error, dt):
        """
        pid control for rover movement
        """
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
        if abs(heading_error) < self.MIN_ACCEPTABLE_HEADING_ERR:
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
        """
        update heading measured using GPS
        """
        gps_yaw_valid = False
        if self.previous_lat is not None:
            dist_moved, gps_head = self.compute_distance_and_heading(
                self.previous_lat, self.previous_long, self.lat, self.long
            )
            # check if enough distance has been driven
            if dist_moved > self.MIN_DIST_FOR_HEADING:
                gps_heading_est = gps_head
                gps_yaw_valid = True

        self.previous_lat = self.lat
        self.previous_long = self.long

        if gps_yaw_valid:
            self.current_heading = self.apply_kalman_filter(
                gps_heading_est, self.imu_yaw
            )
        else:
            self.current_heading = self.imu_yaw

        heading_error = self.target_heading - self.current_heading

        # make sure heading error is between -pi and +pi
        if heading_error > math.pi:
            heading_error -= 2 * math.pi
        elif heading_error < -math.pi:
            heading_error += 2 * math.pi

        rospy.loginfo(
            f"[DRIVER] Dist: {self.distance_to_target:.2f}m | Heading Err: {math.degrees(heading_error):.2f}"
        )

        # calculate what the target heading of the IMU should be
        self.target_yaw = self.imu_yaw + heading_error
        # make sure target yaw error is between -pi and +pi
        if self.target_yaw > math.pi:
            self.target_yaw -= 2 * math.pi
        elif self.target_yaw < -math.pi:
            self.target_yaw += 2 * math.pi

        return heading_error

    def avoid(self, twist):
        """
        check if object too close to robot in front and updates twist accordingly

        returns if an object has been avoided (rover has turned)
        """
        # if no object drive forward
        if self.object == Object.NOBJECT:
            twist.linear.x = 0.4
            twist.angular.z = 0
            self.object_counter = 0
            return False
        # if self.object has switched from .LEFT to .RIGHT or the other way around
        # keep turning right until no object can be seen
        elif self.object_counter > 2:
            twist.linear.x = 0.0
            twist.angular.z = -0.4
        # if object is seen left, turn right
        elif self.object == Object.LEFT:
            if self.prev_object == Object.RIGHT:
                self.object_counter += 1
            twist.linear.x = 0.0
            twist.angular.z = -0.4
        # if object is seen right, turn left
        else:
            if self.prev_object == Object.LEFT:
                self.object_counter += 1
            twist.linear.x = 0.0
            twist.angular.z = 0.4

        return True

    def start_heading_timer(self) -> None:
        self.previous_long = self.long
        self.previous_lat = self.lat
        # set heading timer to two seconds
        self.heading_timer = 2

    def stop_heading_timer(self) -> None:
        self.heading_timer = -1

    def update_heading_timer(self, dt) -> None:
        if self.heading_timer == -1:
            return

        self.heading_timer = max(self.heading_timer - dt, 0)

    def rotate(self) -> float:
        """
        rotate the robot based on current heading and target heading of the IMU

        keeps rotating rover until the current IMU heading is less than 5 degrees
        away from the target IMU heading
        """
        if abs(self.imu_yaw - self.target_yaw) < (math.pi / 180 * 5):
            self.is_updating_heading = False
            self.start_heading_timer()
            return 0

        # calculate which way the rover should be turning
        diff = self.target_yaw - self.imu_yaw
        angular_z = 0.2 if ((diff + math.pi) % (2 * math.pi) - math.pi) > 0 else -0.2

        return angular_z

    def drive(self, _):
        """
        main of Driver, runs every 0.1 seconds
        """
        # so it runs after the user has set up the route and pushed the start button
        if os.environ.get("archebot_start") != "true":
            return
        # get the target coordinates from route.gpx if drive() is ran for first time
        if self.first_time:
            self.first_time = False
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
            self.update_gps_target()
            return

        self.update_heading_timer(dt)

        # check if object was avoided
        avoided = self.avoid(twist)
        self.prev_object = self.object

        # if object was avoided return early
        if avoided:
            self.stop_heading_timer()
            self.cmd_pub.publish(twist)
            return

        # if heading timer is off start the timer
        if self.heading_timer == -1:
            self.start_heading_timer()
            self.cmd_pub.publish(twist)
            return
        # don't update heading if timer is not zero
        elif self.heading_timer != 0:
            self.cmd_pub.publish(twist)
            return

        # update rotation depending on the heading error
        heading_error = self.update_heading()

        # start rotating if heading_error is too big
        if abs(heading_error) > self.MIN_ACCEPTABLE_HEADING_ERR:
            self.stop_heading_timer()
            self.is_updating_heading = True
            return

        self.start_heading_timer()
        self.cmd_pub.publish(twist)
