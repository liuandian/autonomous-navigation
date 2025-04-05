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

rospy.init_node("ocr_lidar_box_center_node")

# 1️⃣ GPU enabled for EasyOCR
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

def camera_info_callback(info):
    global camera_intrinsics
    camera_intrinsics = np.array(info.K).reshape(3, 3)

rospy.Subscriber("/front/camera_info", CameraInfo, camera_info_callback)

def lidar_callback(pc2_msg):
    global lidar_points
    lidar_points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_msg, remove_nans=True)

rospy.Subscriber("/mid/points", PointCloud2, lidar_callback)

def estimate_normal(points):
    pca = PCA(n_components=3)
    pca.fit(points)
    return pca.components_[-1]

def adjust_normal_direction(normal_vector, box_center_point):
    if np.dot(normal_vector, box_center_point) < 0:
        normal_vector = -normal_vector  
    return normal_vector

# 6️⃣ Reduce TF lookup frequency
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

    # 2️⃣ Vectorized point transformation
    lidar_cam = transform_points(lidar_points, transform)

    # 3️⃣ Pre-filter points to front of camera
    valid_indices = lidar_cam[:, 2] > 0
    lidar_cam = lidar_cam[valid_indices]
    lidar_filtered = lidar_points[valid_indices]

    # 4️⃣ Vectorized projection
    uv = (camera_intrinsics @ lidar_cam.T).T
    uv[:, :2] /= lidar_cam[:, 2:3]
    uv = uv[:, :2].astype(int)

    results = ocr_reader.readtext(img_rgb, allowlist='0123456789')

    for bbox, text, conf in results:
        if conf > 0.5 and text.strip() in "123456789":
            rospy.loginfo(f"Detected text: {text} with confidence: {conf}")
            bbox = np.array(bbox, dtype=int)
            u_center = int(np.mean(bbox[:, 0]))
            v_center = int(np.mean(bbox[:, 1]))
            u_min, v_min = np.min(bbox[:, 0]), np.min(bbox[:, 1])
            u_max, v_max = np.max(bbox[:, 0]), np.max(bbox[:, 1])

            in_bbox_mask = (uv[:, 0] >= u_min) & (uv[:, 0] <= u_max) & (uv[:, 1] >= v_min) & (uv[:, 1] <= v_max)
            matched_points = lidar_filtered[in_bbox_mask]

            for u_p, v_p in uv[in_bbox_mask]:
                cv2.circle(img_rgb, (u_p, v_p), 3, (255, 0, 0), -1)

            if len(matched_points) > 15:
                box_center_point = matched_points.mean(axis=0)
                normal_vector = adjust_normal_direction(estimate_normal(matched_points), box_center_point)
                box_center = box_center_point + normal_vector * 0.4

                center_pt = PointStamped()
                center_pt.header.frame_id = "velodyne"
                center_pt.point.x, center_pt.point.y, center_pt.point.z = box_center

                try:
                    transform_world = tf_buffer.lookup_transform("map", "velodyne", rospy.Time(0), rospy.Duration(1.0))
                    center_world = tf2_geometry_msgs.do_transform_point(center_pt, transform_world)
                    rospy.loginfo(f"Digit [{text}] Box center at world: ({center_world.point.x:.2f}, {center_world.point.y:.2f}, {center_world.point.z:.2f})")

                    center_camera = tf2_geometry_msgs.do_transform_point(center_pt, transform)
                    x, y, z = center_camera.point.x, center_camera.point.y, center_camera.point.z

                    if z > 0:
                        uv_center = camera_intrinsics @ np.array([x, y, z]) / z
                        u, v = int(uv_center[0]), int(uv_center[1])
                        cv2.circle(img_rgb, (u, v), 5, (0, 255, 255), -1)
                        cv2.putText(img_rgb, f"{text}@({center_world.point.x:.2f},{center_world.point.y:.2f})", (u, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                        cv2.rectangle(img_rgb, tuple(bbox[0]), tuple(bbox[2]), (0, 255, 0), 2)
                        cv2.circle(img_rgb, (u_center, v_center), 5, (255, 255, 255), -1)
                except Exception as e:
                    rospy.logerr(f"World transform failed: {str(e)}")
            else:
                rospy.logwarn("Not enough inliers for box estimation")

    annotated_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    annotated_msg = ros_numpy.msgify(Image, annotated_bgr, encoding="bgr8")
    annotated_msg.header = img_msg.header
    image_pub.publish(annotated_msg)

    compressed_msg = CompressedImage()
    compressed_msg.header = img_msg.header
    compressed_msg.format = "jpeg"
    compressed_msg.data = np.array(cv2.imencode('.jpg', annotated_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 50])[1]).tobytes()
    compressed_image_pub.publish(compressed_msg)

    end_time = time.time()
    rospy.loginfo(f"Processing time: {end_time - start_time:.2f} seconds")

rospy.Subscriber("/front/image_raw", Image, image_callback)
rospy.spin()