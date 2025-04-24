from ultralytics import YOLO
import os
import ros_numpy

cwd = os.path.dirname(os.path.abspath(__file__))
model = YOLO(os.path.join(cwd, "../include/model.pt"))

iterations = -1
found_shard = True

def save_location(data) -> None:
    global found_shard
    if not found_shard:
        return
    found_shard = False

    print(data)

def shard_detection(data) -> None:
    global iterations
    iterations += 1
    if iterations % 60:
        return
    
    img = ros_numpy.numpify(data)
    results = model.predict(img)
    if 0 != len(results):
        global found_shard
        found_shard = True
