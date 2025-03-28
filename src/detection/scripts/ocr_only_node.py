#!/usr/bin/env python3
import time
import rospy
import ros_numpy
import cv2
import easyocr
from sensor_msgs.msg import Image

# Initialize ROS
rospy.init_node("ocr_only_node")
time.sleep(1)

# Publisher
det_image_pub = rospy.Publisher("/ultralytics/detection/image", Image, queue_size=5)

# OCR setup
ocr_reader = easyocr.Reader(['en'], gpu=False)

def callback(data):
    array = ros_numpy.numpify(data)                     # ROS Image → NumPy (BGR)
    rgb_img = cv2.cvtColor(array, cv2.COLOR_BGR2RGB)    # Convert to RGB for OCR
    annotated_img = rgb_img.copy()

    if det_image_pub.get_num_connections():
        ocr_results = ocr_reader.readtext(rgb_img)

        for (bbox, text, conf) in ocr_results:
            if conf > 0.5 and text.strip().isdigit():
                # Draw bounding box and label
                top_left = tuple(map(int, bbox[0]))
                bottom_right = tuple(map(int, bbox[2]))

                cv2.rectangle(annotated_img, top_left, bottom_right, (255, 0, 255), 2)
                cv2.putText(
                    annotated_img,
                    f"Digit: {text}",
                    (top_left[0], top_left[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (255, 255, 0),
                    2,
                )

        # Convert to BGR for RViz and resize
        bgr_annotated = cv2.cvtColor(annotated_img, cv2.COLOR_RGB2BGR)
        resized = cv2.resize(bgr_annotated, (640, 480))
        det_image_pub.publish(ros_numpy.msgify(Image, resized, encoding="bgr8"))

    rospy.loginfo("OCR frame processed: %s", rospy.get_time())

rospy.Subscriber("/front/image_raw", Image, callback)
rospy.spin()
