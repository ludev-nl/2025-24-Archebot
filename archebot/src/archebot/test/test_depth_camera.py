import os
import random

import cv2
import numpy as np

from src.depth_camera_functions import array_to_image, cam_to_array


class FakeImage:
    def __init__(self):
        self.data = bytes([random.randint(0, 255) for _ in range(614400)])
        self.width = 640
        self.height = 480


def test_cam_to_array():
    fake_data = FakeImage()
    array = cam_to_array(fake_data)

    for height in range(fake_data.height):
        for width in range(fake_data.width):
            # get number from two 8 bit bytestrings
            raw_pixel = int.from_bytes(
                (
                    fake_data.data[
                        (((height * fake_data.width) + width) * 2) : (
                            ((height * fake_data.width) + width) * 2
                        )
                        + 2
                    ]
                ),
                byteorder="little",
            )
            array_pixel = array[height][width]

            assert raw_pixel == array_pixel


def test_array_to_image():
    dummy_data = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
    path = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(path, "array_to_image_unit_test.png")
    array_to_image(path, dummy_data)
    ar = cv2.imread(path)

    # scaled pixels with RGB channels
    test_data = np.array(
        [
            [[28, 28, 28], [56, 56, 56], [85, 85, 85]],
            [[113, 113, 113], [141, 141, 141], [170, 170, 170]],
            [[198, 198, 198], [226, 226, 226], [255, 255, 255]],
        ]
    )

    assert np.array_equal(ar, test_data)

