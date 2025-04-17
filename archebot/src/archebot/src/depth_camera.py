import os

from sensor_msgs.msg import Image

from depth_camera_functions import array_to_image, cam_to_array


def main(data: Image) -> None:
    numpdata = cam_to_array(data)
    path = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(path, "test.png")
    array_to_image(path, numpdata)
