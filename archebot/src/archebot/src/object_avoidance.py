from depth_camera_functions import cam_to_array
import numpy as np

class Avoider:
    def __init__(self):
        self.initialized = 0
        self.corrected = np.zeros((240, 640))
        self.camera_height = 330
        self.boundary = 40
        self.pixel_angle = ((58/480)/360) * 2 * np.pi

    def initialize(self, data):
        # so it only runs once
        if self.initialized > 2:
            return

        # numpy array of data
        arr = cam_to_array(data)
        # angle of middle pixel
        angle = np.arccos(self.camera_height / arr[239, 320])
        # under array
        #            camera_height
        #  ----------------------------------
        #  under (= cos(angle + extra_angle))
        under = np.zeros((240, 640))
        
        # loop over rows
        for i, row in enumerate(arr[:240][::-1]):
            under[239-i, 320] = np.cos(angle + (i * self.pixel_angle))

        # length of max height of objects
        corrected_boundary = self.boundary / under
        # all full lengths of pixels
        pixel_len = self.camera_height / under

        self.initialized += 1
