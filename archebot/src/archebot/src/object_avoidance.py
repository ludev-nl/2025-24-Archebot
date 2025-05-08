from depth_camera_functions import cam_to_array, array_to_image
from std_msgs.msg import String
import numpy as np
import copy
import rospy

class Avoider:
    def __init__(self):
        self.initialized = False
        self.corrected = np.zeros((100, 640))
        self.camera_height = 330
        self.boundary = 40
        self.pixel_angle = ((58/480)/180) * np.pi
        self.highest_idx = None
        self.lowest_idx = None
        # amount of rows of pixels used for object avoidance 
        # 0: is the height of the raised ground, 
        # 1: cut-off amount on the left, 
        # 2: absolute value is the cut-off on the right.
        self.view = [50,(195,-140)] 
        self.publisher = rospy.Publisher("object_detection", String, queue_size=10)


    def detect_object(self, data):
        arr = cam_to_array(data)
        show = copy.deepcopy(arr)[:, 50:]

        show[self.lowest_idx] = 0
        show[self.highest_idx] = 0

        # saves the array to a .npy binary and .png image
        # results can be found in ~/.ros/
        # np.save("arr.npy", show)
        # array_to_image("arr.png", show)
        if np.any(self.corrected[:, 195:-140] - arr[self.lowest_idx:self.highest_idx, 195:-140] > 0): # detect only exactly in front
            self.publisher.publish("OBJECT")
        else:
            self.publisher.publish("NOBJECT")

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

