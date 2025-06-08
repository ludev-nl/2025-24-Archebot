from ultralytics import YOLO
import os
import cv2

cwd = os.path.dirname(os.path.abspath(__file__))
model = YOLO(os.path.join(cwd, "../include/model.pt"))

# Read YOLO labels position labels and count amount of boxes
def read_count(labelname: str) -> int:
    count = 0
    try:
        with open(labelname, "r") as f:
            for line in f.readlines():
                count += 1
    except Exception:
        return None
    return count

# Test the amount of shards detected with the true amount of shards
def helper_detection_count(imgname: str, labelname: str) -> None:
    # Load image
    img = cv2.imread(imgname)
    assert img is not None, f"Image '{imgname}' could not be loaded"

    # Get true count from filename
    true_count = read_count(labelname)
    assert true_count is not None, "A valid label file must be given"

    # Run detection
    pred_count = len(model.predict(source=img, conf=0.6)[0].boxes)

    # Check predicted with true
    assert pred_count == true_count, f"Predicted {pred_count}, but expected {true_count}"

def test_detection_count():
    helper_detection_count(os.path.join(cwd, "images/test_detection_0.jpeg"), os.path.join(cwd, "labels/test_detection_0.txt"))
    helper_detection_count(os.path.join(cwd, "images/test_detection_1.jpeg"), os.path.join(cwd, "labels/test_detection_1.txt"))
    helper_detection_count(os.path.join(cwd, "images/test_detection_2.jpeg"), os.path.join(cwd, "labels/test_detection_2.txt"))
    helper_detection_count(os.path.join(cwd, "images/test_detection_3.jpeg"), os.path.join(cwd, "labels/test_detection_3.txt"))
