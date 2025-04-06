#!/usr/bin/env python
import time
import sys
import numpy as np

# Dynamic patch for numpy float deprecation
if not hasattr(np, 'float'):
    np.float = float

import ros_numpy
import rospy
import cv2
from sensor_msgs.msg import Image
from ultralytics import YOLO
import os

# Load the YOLO model 
script_dir = os.path.dirname(os.path.abspath(__file__))

# Construct the model path relative to the script directory
model_path = os.path.join(script_dir, "yolo_3.pt")

detection_model = YOLO(model_path)

# Initialize ROS
rospy.init_node("ultralytics")
time.sleep(1)

# Publisher for annotated images
det_image_pub = rospy.Publisher("/ultralytics/detection/image", Image, queue_size=5)

def callback(data):
    """Callback function to process image and publish annotated images."""
    try:
        # Convert ROS Image to NumPy array (BGR format from ROS)
        array = ros_numpy.numpify(data)

        # Convert BGR to RGB for YOLO inference
        rgb_img = cv2.cvtColor(array, cv2.COLOR_BGR2RGB)

        # Perform inference with the YOLO model
        det_result = detection_model(rgb_img, conf=0.5)  # Set confidence threshold here
        #det_result = detection_model(rgb_img)

        
        
        # Retrieve the plot from the result (which might be RGB already)
        det_annotated = det_result[0].plot(show=False)

        # Check the output format of the annotated image
        if det_annotated.shape[2] == 3:
            # Directly use the output if it is already BGR
            bgr_annotated = det_annotated
        else:
            # Convert from RGB to BGR if necessary
            bgr_annotated = cv2.cvtColor(det_annotated, cv2.COLOR_RGB2BGR)

        # Publish the annotated image with correct encoding
        det_image_pub.publish(ros_numpy.msgify(Image, bgr_annotated, encoding="bgr8"))

        rospy.loginfo(f"Processed frame at time: {rospy.get_time()}")

    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")

# ROS subscriber for the image topic
rospy.Subscriber("/front/image_raw", Image, callback)

# Start ROS event loop
rospy.spin()
