from ultralytics import YOLO
import cv2

model = YOLO("../include/best.pt")

# Read YOLO labels position labels and convert to pixel coords.
def read_boxes(labelname: str, img_width: int, img_height: int) -> None | list[tuple[int, int, int, int]]:
    boxes = []
    try:
        with open(labelname, "r") as f:
            for line in f.readlines():
                _, x1, y1, x2, y2, x3, y3, x4, y4 = map(float, line.strip().split())
                px = [int(x * img_width) for x in (x1, x2, x3, x4)]
                py = [int(y * img_height) for y in (y1, y2, y3, y4)]
                boxes.append((min(px), min(py), max(px), max(py)))
    except:
        return None
    return boxes

# Calculate IOU score of two bounding boxes (overlap)
def iou(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
    a_x = max(a[0], b[0])
    a_y = max(a[1], b[1])
    b_x = min(a[2], b[2])
    b_y = min(a[3], b[3])

    inter_area = max(0, b_x - a_x) * max(0, b_y - a_y)
    a_area = (a[2] - a[0]) * (a[3] - a[1])
    b_area = (b[2] - b[0]) * (b[3] - b[1])

    return inter_area / float(a_area + b_area - inter_area + 1e-6)

# Test the predicted bounding boxes with the true bounding boxes
def helper_detection_box(imgname: str, labelname: str, iou_threshold=0.5):
    # Load image
    img = cv2.imread(imgname)
    assert img is not None, f"Image '{imgname}' could not be loaded"
    h, w = img.shape[:2]

    # Load label boxes (converted to pixel coords)
    true_boxes = read_boxes(labelname, w, h)
    assert true_boxes is not None, f"Labels '{labelname}' could not be loaded"

    # Run YOLO prediction
    results = model.predict(img, conf=0.6)
    pred_boxes = results[0].boxes.xyxy.cpu().numpy().astype(int)
    pred_boxes = [tuple(box) for box in pred_boxes]
    assert len(pred_boxes) == len(true_boxes), f"Predicted {len(pred_boxes)} boxes, but expected {len(true_boxes)} boxes"

    # Match predicted boxes with true boxes via IOU
    for true_box in true_boxes:
        max_score = 0.0
        for pred_box in pred_boxes:
            score = iou(true_box, pred_box)
            if score > max_score:
                max_score = score
            if score >= iou_threshold:
                break
        else:
            assert max_score >= iou_threshold, "IOU of {max_score} of predicted box is lower than threshold {iou_threshold}"

def test_detection_box():
    helper_detection_box("images/test_detection_0.jpeg", "labels/test_detection_0.txt")
    helper_detection_box("images/test_detection_1.jpeg", "labels/test_detection_1.txt")
    helper_detection_box("images/test_detection_2.jpeg", "labels/test_detection_2.txt")
    helper_detection_box("images/test_detection_3.jpeg", "labels/test_detection_3.txt")
