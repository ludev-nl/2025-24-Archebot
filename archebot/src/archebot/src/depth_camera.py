import traceback
import numpy as np
import cv2
import os

def main(data) -> None:
    path = os.path.dirname(os.path.abspath(__file__))
    numpdata = cam_to_array(data)
    path = os.path.join(path,"test.png")
    array_to_image(path, numpdata)

def array_to_image(path: str, numpdata: np.array) -> None:
    """
    Writes camera data to image file
    Input: path: string, array: numpy array
    Output: None (image file in directory)
    """
    try:
        cv2.imwrite(path, numpdata)
    except Exception as e:
        print(e)
        print(traceback.format_exc())

def cam_to_array(data) -> np.array:
    """
    Writes camera data to numpy array
    Input: data:
    Output: numpdata: np.array
    """
    print(type(data))
    input()
    numpdata = np.frombuffer(data.data, dtype=np.uint16)
    numpdata = (numpdata/max(numpdata)*255).astype("uint8")
    numpdata = numpdata.reshape(data.height, data.width)

    return numpdata