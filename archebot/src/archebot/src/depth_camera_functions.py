import traceback

import cv2
import numpy as np


def array_to_image(path: str, numpdata: np.array) -> None:
    """
    Writes numpy array to image file
    Input: path: string, array: numpy array
    Output: None (image file in directory)
    """
    numpdata = (numpdata / np.max(numpdata) * 255).astype(np.uint8)
    try:
        cv2.imwrite(path, numpdata)
    except Exception as e:
        print(e)
        print(traceback.format_exc())


def cam_to_array(data) -> np.array:
    """
    Writes camera data to numpy array
    Input: data: sensor_msgs/Image
    Output: numpdata: np.array
    """
    numpdata = np.frombuffer(data.data, dtype=np.uint16)
    numpdata = numpdata.reshape(data.height, data.width)

    return numpdata
