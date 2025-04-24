from PIL import Image
from ultralytics import YOLO
import os
import ros_numpy

cwd = os.path.dirname(os.path.abspath(__file__))
model = YOLO(os.path.join(cwd, "../include/model.pt"))

gps_iter = -1
camera_iter = -1
shard_img = None

NDEBUG = False
if not NDEBUG:
    result_count = 0

def save_location(data) -> None:
    global gps_iter
    gps_iter += 1
    if gps_iter % 60:
        return
    global shard_img
    if shard_img is None:
        return
    shard_img.save(f"archebot/{data.longitude},{data.latitude}")
    shard_img = None

def shard_detection(data) -> None:
    global camera_iter
    camera_iter += 1
    if camera_iter % 60:
        return
    img = ros_numpy.numpify(data)[:,:,::-1]
    global shard_img
    if shard_img is None:
        if NDEBUG:
            results = model.predict(img, stream=True, verbose=False)
        else:
            results = model.predict(img, name="tmp", save=True, stream=True, project="archebot", verbose=True)
        for result in results:
            if not NDEBUG:
                while not os.path.exists("archebot/tmp/image0.jpg"):
                    pass
                global result_count
                os.rename("archebot/tmp/image0.jpg", f"archebot/{result_count}.jpg")
                os.rmdir("archebot/tmp")
                result_count += 1
            if 0 != len(result.boxes):
                shard_img = Image.fromarray(img)
                break
