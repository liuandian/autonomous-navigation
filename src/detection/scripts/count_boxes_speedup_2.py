#!/usr/bin/env python3
import rospy
import ros_numpy
import numpy as np
import cv2
import easyocr
import tf2_ros
import tf2_geometry_msgs
import tf.transformations
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped
from sklearn.decomposition import PCA
import time
import math

rospy.init_node("ocr_lidar_box_center_node")

ocr_reader = easyocr.Reader(['en'], gpu=True)
rospy.loginfo("EasyOCR initialized. Using GPU: {}".format(ocr_reader.device == 'cuda'))

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

lidar_points = None
camera_intrinsics = None
transform = None
last_tf_update = rospy.Time(0)

image_pub = rospy.Publisher("/detection/image_annotated", Image, queue_size=1)
compressed_image_pub = rospy.Publisher("/detection/image_annotated/compressed", CompressedImage, queue_size=1)

# Dictionary to store counted box centers and counts
box_centers = {}
box_counts = {}

def is_new_box(digit, center, threshold=2.0):
    """
    Check if the detected box is a new instance based on its center coordinates.
    """
    if digit not in box_centers:
        box_centers[digit] = []
        box_counts[digit] = 0

    for prev_center in box_centers[digit]:
        distance = math.sqrt((prev_center[0] - center[0]) ** 2 + 
                             (prev_center[1] - center[1]) ** 2 + 
                             (prev_center[2] - center[2]) ** 2)
        if distance < threshold:
            return False
    box_centers[digit].append(center)
    box_counts[digit] += 1
    return True

def camera_info_callback(info):
    global camera_intrinsics
    camera_intrinsics = np.array(info.K).reshape(3, 3)

rospy.Subscriber("/front/camera_info", CameraInfo, camera_info_callback)

def lidar_callback(pc2_msg):
    global lidar_points
    lidar_points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_msg, remove_nans=True)

rospy.Subscriber("/mid/points", PointCloud2, lidar_callback)

def update_transform():
    global transform, last_tf_update
    now = rospy.Time.now()
    if (now - last_tf_update).to_sec() > 1.0:
        try:
            transform = tf_buffer.lookup_transform("front_camera_optical", "velodyne", rospy.Time(0), rospy.Duration(1.0))
            last_tf_update = now
        except Exception as e:
            rospy.logwarn(f"TF lookup failed: {str(e)}")

def transform_points(points, transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    rot_mat = tf.transformations.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])[:3, :3]
    trans_vec = np.array([translation.x, translation.y, translation.z])
    return points @ rot_mat.T + trans_vec

def image_callback(img_msg):
    start_time = time.time()
    global lidar_points, camera_intrinsics, transform

    if lidar_points is None or camera_intrinsics is None:
        rospy.logwarn("Waiting for LiDAR points or camera intrinsics...")
        return

    update_transform()
    if transform is None:
        return

    img = ros_numpy.numpify(img_msg)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    results = ocr_reader.readtext(img_rgb, allowlist='0123456789')

    for bbox, text, conf in results:
        if conf > 0.5 and text.strip().isdigit():
            bbox = np.array(bbox, dtype=int)
            u_center = int(np.mean(bbox[:, 0]))
            v_center = int(np.mean(bbox[:, 1]))

            center_pt = PointStamped()
            center_pt.header.frame_id = "velodyne"
            center_pt.point.x, center_pt.point.y, center_pt.point.z = u_center, v_center, 0

            try:
                # Transform from velodyne to base_link instead of map
                transform_base = tf_buffer.lookup_transform("base_link", "velodyne", rospy.Time(0), rospy.Duration(1.0))
                center_base = tf2_geometry_msgs.do_transform_point(center_pt, transform_base)
                box_center_coords = (center_base.point.x, center_base.point.y, center_base.point.z)

                if is_new_box(text, box_center_coords):
                    count = box_counts[text]
                    rospy.loginfo(f"Digit [{text}] Box center at base_link: {box_center_coords} - Counted ({count})")
                else:
                    rospy.loginfo(f"Digit [{text}] Box center at base_link: {box_center_coords} - Duplicate, not counted")

            except Exception as e:
                rospy.logerr(f"Base_link transform failed: {str(e)}")

    annotated_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    annotated_msg = ros_numpy.msgify(Image, annotated_bgr, encoding="bgr8")
    annotated_msg.header = img_msg.header
    image_pub.publish(annotated_msg)

    end_time = time.time()
    rospy.loginfo(f"Processing time: {end_time - start_time:.2f} seconds")

rospy.Subscriber("/front/image_raw", Image, image_callback)
rospy.spin()
