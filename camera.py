from cv_bridge import CvBridge
import cv2

BRIDGE = CvBridge()

# FULL path needed to work
# Download from: https://github.com/opencv/opencv/tree/master/data/haarcascades
face_cascade = cv2.CascadeClassifier('/home/gustav/ros_ws/src/object_detection/scripts/haarcascade_frontalface_default.xml')


def main(data) -> None:
    cv_img = BRIDGE.imgmsg_to_cv2(data, "passthrough")

    gray_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray_image, scaleFactor=1.1, minNeighbors=5)

    print(faces)
    # for (x, y, w, h) in faces:
    #     cv2.rectangle(cv_img, (x, y), (x+w, y+h), (255, 0, 0), 2)
    # cv2.imshow("Camera", cv_img)
    # cv2.waitKey(1)
