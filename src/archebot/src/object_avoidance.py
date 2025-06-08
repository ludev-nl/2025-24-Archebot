import os
import sys

import numpy as np
import rospy
from depth_camera_functions import cam_to_array
from enums import Object
from std_msgs.msg import UInt8


class Avoider:
    def __init__(self):
        # CONSTANTS:
        self.CAMERA_HEIGHT = 330  # in mm
        self.BOUNDARY = 40  # in mm
        self.PIXEL_ANGLE = ((58 / 480) / 180) * np.pi  # ≈ 0.12 degrees in radians

        # makes sure initialize() is run once
        self.initialized = False
        # boundary for when an object is considered an obstacle
        self.corrected = np.zeros((100, 640))
        # highest and lowest idx of the camera view
        self.highest_idx = None
        self.lowest_idx = None
        # amount of rows of pixels used for object avoidance
        # 0: is the height of the raised ground,
        # 1.1: cut-off amount on the left,
        # 1.2: absolute value is the cut-off on the right.
        self.view = [50, (195, -140)]
        self.view_width = 640 - self.view[1][0] + self.view[1][1]  # in pixels

        section_amount = 2
        self.section_width = self.view_width // section_amount
        # fmt: off
        # the relative view sections
        self.view_sections = [(i * self.section_width, (i + 1) * self.section_width) for i in range(section_amount)]  
        # fmt: on

        # to publish where the Avoider sees objects
        self.publisher = rospy.Publisher("object_detection", UInt8, queue_size=10)
        # counter to limit the amount of times the rover runs detect_object()
        self.counter = 0
        self.object = Object.NOBJECT

    def detect_object(self, data):
        self.counter += 1

        # detect_object is too slow to run every time the depth cam publishes data
        if self.counter < 3:
            return

        self.counter = 0

        arr = cam_to_array(data)

        # fmt: off
        # array view inside the defined boxes
        arr_view = arr[self.lowest_idx : self.highest_idx, self.view[1][0]:self.view[1][1]].copy()
        # fmt: on

        # make all zero's 65535 (uint16 max value) to handle small parts the camera cannot measure
        arr_view[arr_view == 0] = 65535

        # detect only exactly in front
        if np.any(self.corrected[:, self.view[1][0] : self.view[1][1]] - arr_view > 0):
            # fmt: off
            # left and right side of camera view
            corrected_left = self.corrected[:, self.view[1][0] + self.view_sections[0][0]:self.view[1][0] + self.view_sections[0][1]]
            corrected_right = self.corrected[:, self.view[1][0] + self.view_sections[1][0]:self.view[1][0] + self.view_sections[1][1]]

            # if left detects oject
            if np.any(corrected_left - arr_view[:, self.view_sections[0][0] : self.view_sections[0][1]] > 0):
            # fmt: on
                if self.object == Object.LEFT:
                    return
                self.publisher.publish(Object.LEFT.value)
                self.object = Object.LEFT
            # fmt: off
            # right detects object
            elif np.any(corrected_right - arr_view[:, self.view_sections[1][0] : self.view_sections[1][1]] > 0):
            # fmt: on
                if self.object == Object.RIGHT:
                    return
                self.publisher.publish(Object.RIGHT.value)
                self.object = Object.RIGHT

        # no object detected
        else:
            if self.object == Object.NOBJECT:
                return
            self.publisher.publish(Object.NOBJECT.value)
            self.object = Object.NOBJECT

    def initialize(self, data):
        # so it only runs once
        if self.initialized:
            return

        # so it runs after the user has set up the route and pushed the start button
        if os.environ.get("archebot_start") != "true":
            return

        # numpy array of data
        arr = cam_to_array(data)

        higher_than_684 = np.where(arr[:, 320] >= 684)
        if len(higher_than_684[0]) == 0:
            print(
                "NO CALLIBRATION POINT FOUND POSSIBLY TOO CLOSE TO OBJECT IN FRONT",
                file=sys.stderr,
            )
            sys.exit(2)

        # angle of first pixel above 684
        self.highest_idx = np.max(higher_than_684)  # lowest point on screen from camera
        self.lowest_idx = max(
            self.highest_idx - self.view[0], 0
        )  # prevent index going in the negative

        # get median of the row (the measurement of the distance of the lowest point on screen)
        median = np.median(arr[self.highest_idx])
        alpha = self.PIXEL_ANGLE * (self.highest_idx - 240)

        B = median / np.cos(alpha)
        if B == 0:
            print("Calibration failed")
            sys.exit(100)
        angle = np.arccos(
            self.CAMERA_HEIGHT / B
        )  # absolute angle to lowest point on camera that is used
        # angle to the middle of the camera
        angle_to_middle = (
            angle + alpha
        )  # angle to a certain point + the relative angle to the middle

        # under array purpose
        #     (camera_height - view_height)
        #  ---------------------------------- = distance from camera to raised ground = C
        #       cos(angle + extra_angle)
        C = np.zeros((self.view[0], 640))
        self.corrected = np.zeros((self.view[0], 640))

        # loop over rows
        for i, _ in enumerate(arr[self.lowest_idx : self.highest_idx][::-1]):
            relative_angle = i * self.PIXEL_ANGLE
            # the absolute angle to a certain row in the array
            absolute_angle = angle + relative_angle
            C[-i - 1, 320] = (
                (self.CAMERA_HEIGHT - self.BOUNDARY) * 1 / (np.cos(absolute_angle))
            )
            relative_angle2 = absolute_angle - angle_to_middle  # relative_angle
            gamma = 0.5 * np.pi - relative_angle2

            # the distance to the "camera grid"
            self.corrected[-i - 1] = np.sin(gamma) * C[-i - 1, 320]

        self.initialized = True
        print("avoider initialized")
