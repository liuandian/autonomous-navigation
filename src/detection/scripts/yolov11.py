#!/usr/bin/env python
# import time

# import ros_numpy
# import rospy
# from sensor_msgs.msg import Image

# from ultralytics import YOLO

# detection_model = YOLO("yolo11m.pt")
# #segmentation_model = YOLO("yolo11m-seg.pt")
# rospy.init_node("ultralytics")
# time.sleep(1)

# det_image_pub = rospy.Publisher("/ultralytics/detection/image", Image, queue_size=5)
# #seg_image_pub = rospy.Publisher("/ultralytics/segmentation/image", Image, queue_size=5)


# def callback(data):
#     """Callback function to process image and publish annotated images."""
#     array = ros_numpy.numpify(data)
#     if det_image_pub.get_num_connections():
#         det_result = detection_model(array)
#         det_annotated = det_result[0].plot(show=False)
#         det_image_pub.publish(ros_numpy.msgify(Image, det_annotated, encoding="rgb8"))
#     hello_str = "hello world %s" % rospy.get_time()
#     rospy.loginfo(hello_str)


#     # if seg_image_pub.get_num_connections():
#     #     seg_result = segmentation_model(array)
#     #     seg_annotated = seg_result[0].plot(show=False)
#     #     seg_image_pub.publish(ros_numpy.msgify(Image, seg_annotated, encoding="rgb8"))


# rospy.Subscriber("/front/image_raw", Image, callback)

# while True:
#     rospy.spin()

#!/usr/bin/env python
import time

import ros_numpy
import rospy
import cv2
from sensor_msgs.msg import Image
from ultralytics import YOLO

detection_model = YOLO("yolo11m.pt")
rospy.init_node("ultralytics")
time.sleep(1)

det_image_pub = rospy.Publisher("/ultralytics/detection/image", Image, queue_size=5)

def callback(data):
    """Callback function to process image and publish annotated images."""
    array = ros_numpy.numpify(data)                     # ROS image to NumPy
    rgb_img = cv2.cvtColor(array, cv2.COLOR_BGR2RGB)    

    if det_image_pub.get_num_connections():
        det_result = detection_model(rgb_img)           # 用 RGB 图像推理
        det_annotated = det_result[0].plot(show=False)

        # 反转回来给 RViz 显示（RViz 默认要 BGR）
        bgr_annotated = cv2.cvtColor(det_annotated, cv2.COLOR_RGB2BGR)

        det_image_pub.publish(ros_numpy.msgify(Image, bgr_annotated, encoding="bgr8"))

    hello_str = "hello world %s" % rospy.get_time()
    rospy.loginfo(hello_str)

rospy.Subscriber("/front/image_raw", Image, callback)
rospy.spin()
