from ultralytics import YOLO
import cv2

model = YOLO('best.pt')

# Read YOLO labels position labels and count amount of boxes
def read_count(labelname: str) -> None | int:
    count = 0
    try:
        with open(labelname, "r") as f:
            for line in f.readlines():
                count += 1
    except:
        return None
    return count

# Test the amount of shards detected with the true amount of shards
def test_detection_count(imgname: str, labelname: str) -> None:
    # Load image
    img = cv2.imread(imgname)
    assert img is not None, f"Image '{imgname}' could not be loaded"

    # Get true count from filename
    true_count = read_count(labelname)
    assert true_count is not None, f"Labels '{labelname}' could not be loaded"

    # Run detection
    pred_count = len(model.predict(source=img, conf=0.6)[0].boxes)

    # Check predicted with true
    assert pred_count == true_count, f"Predicted {pred_count}, but expected {true_count}"
