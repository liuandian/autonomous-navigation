#!/usr/bin/env python3
import rospy
import ros_numpy
import numpy as np
import cv2
import easyocr
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import PointStamped
from sklearn.decomposition import PCA
import time

rospy.init_node("ocr_lidar_box_center_node")

# Retrieve parameters with defaults
frame_skip = rospy.get_param('~frame_skip', 3)
max_points = rospy.get_param('~max_points', 5000)
distance_threshold = 0.8

ocr_reader = easyocr.Reader(['en'], gpu=False)
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

lidar_points = None
camera_intrinsics = None
box_count = {}
detected_centers = []

image_pub = rospy.Publisher("/detection/image_annotated", Image, queue_size=5)

# Maintain a global frame counter
frame_count = 0

def camera_info_callback(info):
    global camera_intrinsics
    camera_intrinsics = np.array(info.K).reshape(3, 3)

rospy.Subscriber("/front/camera_info", CameraInfo, camera_info_callback, queue_size=5)

def lidar_callback(pc2_msg):
    global lidar_points, max_points
    points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_msg, remove_nans=True)
    if len(points) > max_points:
        points = points[:max_points]
    lidar_points = points

rospy.Subscriber("/mid/points", PointCloud2, lidar_callback, queue_size=5)

def estimate_normal(points):
    pca = PCA(n_components=3)
    pca.fit(points)
    return pca.components_[-1]

def adjust_normal_direction(normal_vector, box_center_point):
    if np.dot(normal_vector, box_center_point) < 0:
        normal_vector = -normal_vector
    return normal_vector

def is_new_box(center):
    for existing_center in detected_centers:
        if np.linalg.norm(np.array(center) - np.array(existing_center)) < distance_threshold:
            return False
    return True

def image_callback(img_msg):
    global lidar_points, camera_intrinsics, frame_count, frame_skip
    frame_count += 1
    if frame_count % frame_skip != 1:
        return

    start_time = time.time()

    if lidar_points is None or camera_intrinsics is None:
        rospy.logwarn("Waiting for LiDAR points or camera intrinsics...")
        return

    img = ros_numpy.numpify(img_msg)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    try:
        transform = tf_buffer.lookup_transform("front_camera_optical", "velodyne", rospy.Time(0), rospy.Duration(1.0))
    except Exception as e:
        rospy.logerr(f"TF transform lookup failed: {str(e)}")
        return

    transformed_points = []
    for pt in lidar_points:
        pt_stamped = PointStamped()
        pt_stamped.header.frame_id = "velodyne"
        pt_stamped.point.x, pt_stamped.point.y, pt_stamped.point.z = pt.tolist()
        try:
            pt_transformed = tf2_geometry_msgs.do_transform_point(pt_stamped, transform)
            if pt_transformed.point.z <= 0:
                continue
            transformed_points.append(pt)
        except Exception:
            continue

    results = ocr_reader.readtext(img_rgb, allowlist='0123456789')
    for bbox, text, conf in results:
        if conf > 0.3 and text.strip().isdigit():
            bbox = np.array(bbox, dtype=int)
            u_center = int(np.mean(bbox[:, 0]))
            v_center = int(np.mean(bbox[:, 1]))
            rospy.loginfo(f"Detected text: {text} with confidence: {conf}")

            if len(transformed_points) > 15:
                box_center_point = np.mean(transformed_points, axis=0)
                if is_new_box(box_center_point):
                    detected_centers.append(box_center_point)
                    box_count[text] = box_count.get(text, 0) + 1
                    rospy.loginfo(f"Box {text} count: {box_count[text]}")

    annotated_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    annotated_msg = ros_numpy.msgify(Image, annotated_bgr, encoding="bgr8")
    annotated_msg.header = img_msg.header
    image_pub.publish(annotated_msg)
    rospy.loginfo(f"Frame processing time: {time.time() - start_time:.3f} seconds")

rospy.Subscriber("/front/image_raw", Image, image_callback, queue_size=5)
rospy.spin()
