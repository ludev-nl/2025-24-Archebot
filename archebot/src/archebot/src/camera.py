from ultralytics import YOLO
import os
import ros_numpy


model = YOLO("../include/best.pt")


def main(data) -> None:
    img = ros_numpy.numpify(data)
    results = model.predict(img)
    print(results)
