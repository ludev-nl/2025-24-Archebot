from depth_camera_functions import cam_to_array, array_to_image
from std_msgs.msg import String
import numpy as np
import rospy
import time

class Avoider:
    def __init__(self):
        self.initialized = False
        self.corrected = np.zeros((100, 640))
        self.camera_height = 330
        self.boundary = 40 # in mm
        self.pixel_angle = ((58/480)/180) * np.pi
        self.highest_idx = None
        self.lowest_idx = None
        # amount of rows of pixels used for object avoidance 
        # 0: is the height of the raised ground, 
        # 1: cut-off amount on the left, 
        # 2: absolute value is the cut-off on the right.
        self.view = [50,(195,-140)] 
        self.publisher = rospy.Publisher("object_detection", String, queue_size=10)
        self.counter = 0
        self.object = False


    @staticmethod
    def median_filter(arr: np.ndarray, FILLIN: int) -> None:
        """
        For each row in `arr`, replaces each entry in every non-overlapping chunk
        of length FILLIN with the median of that chunk
        """
        rows, cols = arr.shape
        blocks = cols // FILLIN

        # make view (rows * blocks * FILLIN)
        reshaped = arr.reshape(rows, blocks, FILLIN)

        # compute median for every block
        medians = np.median(reshaped, axis=2)

        # repeat every median FILLIN times in a new dimension
        filled = np.repeat(medians.reshape(rows, blocks, 1), FILLIN, axis=2)

        # write the view back to the original array (rows * cols)
        arr[:, :cols] = filled.reshape(rows, cols)


    def detect_object(self, data):
        self.counter += 1

        if self.counter < 3:
            return

        self.counter = 0

        arr = cam_to_array(data)

        # saves the array to a .npy binary and .png image
        # results can be found in ~/.ros/
        # show = arr[:, 50:].copy()
        # np.save("arr.npy", show)
        # array_to_image("arr.png", show)

        arr_view = arr[self.lowest_idx:self.highest_idx, 195:-140].copy()
        self.median_filter(arr_view, 61)
        if np.any(self.corrected[:, 195:-140] - arr_view > 0): # detect only exactly in front
            if not self.object:
                self.publisher.publish("OBJECT")
                self.object = True
        else:
            if self.object:
                self.publisher.publish("NOBJECT")
                self.object = False

    def initialize(self, data):
        # so it only runs once
        if self.initialized:
            return

        # numpy array of data
        arr = cam_to_array(data)

        # angle of first pixel above 684
        self.highest_idx = np.max(np.where(arr[:, 320] >= 684)) # lowest point on screen from camera
        self.lowest_idx = max(self.highest_idx-self.view[0], 0) # prevent index going in the negative # highest point on screen from camera
        # get median of the row (the measurement of the distance of the lowest point on screen)
        median = np.median(arr[self.highest_idx])
        alpha = self.pixel_angle * (self.highest_idx-240)
        B =  median/np.cos(alpha)
        angle = np.arccos(self.camera_height / B) # absolute angle to lowest point on camera that is used
        # angle to the middle of the camera
        angle_to_middle = angle + alpha # angle to a certain point + the relative angle to the middle
        

        # print(arr[self.highest_idx, 320])
        # print(self.highest_idx)

        # under array purpose
        #     (camera_height - view_height)
        #  ---------------------------------- = distance from camera to raised ground = C
        #       cos(angle + extra_angle)
        C = np.zeros((self.view[0], 640))
        self.corrected = np.zeros((self.view[0], 640))

        # loop over rows
        for i, row in enumerate(arr[self.lowest_idx:self.highest_idx][::-1]):
            relative_angle = i * self.pixel_angle
            absolute_angle = angle + relative_angle # the absolute angle to a certain row in the array
            C[-i - 1, 320] = (self.camera_height-self.boundary) * 1/(np.cos(absolute_angle))
            relative_angle2 = absolute_angle - angle_to_middle # relative_angle
            gamma = 0.5*np.pi - relative_angle2
            # self.corrected[-i - 1, 320] = np.sin(gamma) * C[-i - 1, 320]
            self.corrected[-i -1] = np.sin(gamma) * C[-i - 1, 320] # the distance to the "camera grid"

        self.initialized = True
        print("avoider initialized")

