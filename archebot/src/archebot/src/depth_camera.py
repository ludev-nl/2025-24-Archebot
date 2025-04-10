import traceback
import numpy as np
import cv2
import time

def main(data) -> None:
    try:
        input("op pauze")
        print(type(data))

        numpdata = cam_to_array(data)
        
        cv2.imwrite("/home/pi/test.png", numpdata)
        print("written")
        
    except Exception as e:
        print(e)
        print(traceback.format_exc())
        print("exeption AAAAAAAAA")
        pass


def cam_to_array(data) -> np.array:
    numpdata = np.frombuffer(data.data, dtype=np.uint16)
    numpdata = (numpdata/max(numpdata)*255).astype("uint8")
    numpdata = numpdata.reshape(data.height, data.width)

    return numpdata