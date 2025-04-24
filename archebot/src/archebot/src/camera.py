from ultralytics import YOLO
import os
import ros_numpy

cwd = os.path.dirname(os.path.abspath(__file__))
model = YOLO(os.path.join(cwd, "../include/best.pt"))

def main(data) -> None:
    img = ros_numpy.numpify(data)
    results = model.predict(img)
    print(results)
