#!/usr/bin/env python3
import rospy
import concurrent.futures
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
from collections import defaultdict
from std_msgs.msg import Int8

class BoxCountNode:
    def __init__(self):
        rospy.init_node("ocr_lidar_box_center_node")

        self.ocr_reader = easyocr.Reader(['en'], gpu=True)
        rospy.loginfo(f"EasyOCR initialized. Using GPU: {self.ocr_reader.device == 'cuda'}")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self.ocr_result = None
        self.last_ocr_time = 0
        self.ocr_interval = 0.5  # 仅每0.5秒进行一次OCR

        self.lidar_points = None
        self.camera_intrinsics = None
        self.transform = None
        self.last_tf_update = rospy.Time(0)

        self.image_pub = rospy.Publisher("/detection/image_annotated", Image, queue_size=1)
        self.compressed_image_pub = rospy.Publisher("/detection/image_annotated/compressed", CompressedImage, queue_size=1)
        self.min_box_pub = rospy.Publisher("/detection/min_box_count", Int8, queue_size=1)

        self.finalized_boxes = defaultdict(list)
        self.pending_boxes = defaultdict(list)
        self.box_counts = defaultdict(int)
        self.match_points_num = 20
        self.dist_existing = 1.2
        self.dist_pending = 0.3
        self.required_frames = 20

        rospy.Subscriber("/front/camera_info", CameraInfo, self.camera_info_callback)
        rospy.Subscriber("/mid/points", PointCloud2, self.lidar_callback)
        rospy.Subscriber("/front/image_raw", Image, self.image_callback)

    def camera_info_callback(self, info):
        self.camera_intrinsics = np.array(info.K).reshape(3, 3)

    def lidar_callback(self, pc2_msg):
        self.lidar_points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_msg, remove_nans=True)

    def estimate_normal(self, points):
        pca = PCA(n_components=3)
        pca.fit(points)
        return pca.components_[-1]

    def adjust_normal_direction(self, normal_vector, box_center_point):
        if np.dot(normal_vector, box_center_point) < 0:
            normal_vector = -normal_vector
        return normal_vector

    def update_transform(self, data_time):
        now = rospy.Time.now()
        if (now - self.last_tf_update).to_sec() > 1.0:
            try:
                self.transform = self.tf_buffer.lookup_transform(
                    "front_camera_optical", "velodyne", rospy.Time(0), rospy.Duration(1.0)
                )
                self.last_tf_update = now
            except tf2_ros.ExtrapolationException as e:
                rospy.logwarn(f"TF extrapolation failed: {str(e)}")
                self.transform = None
            except Exception as e:
                rospy.logwarn(f"TF lookup failed: {str(e)}")
                self.transform = None

    def transform_points(self, points, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rot_mat = tf.transformations.quaternion_matrix(
            [rotation.x, rotation.y, rotation.z, rotation.w]
        )[:3, :3]
        trans_vec = np.array([translation.x, translation.y, translation.z])

        transformed = points @ rot_mat.T + trans_vec
        transformed = transformed[~np.isnan(transformed).any(axis=1)]
        return transformed

    def distance_3d(self, c1, c2):
        return math.sqrt((c1[0] - c2[0]) ** 2 + (c1[1] - c2[1]) ** 2 + (c1[2] - c2[2]) ** 2)

    def handle_detection(self, digit, box_center, dist_existing=1.0, dist_pending=0.5, required_frames=4):
        for fc in self.finalized_boxes[digit]:
            if self.distance_3d(fc, box_center) < dist_existing:
                return

        for pbox in self.pending_boxes[digit]:
            last_center = pbox["centers"][-1]
            if self.distance_3d(last_center, box_center) < dist_pending:
                pbox["centers"].append(box_center)
                if len(pbox["centers"]) >= required_frames:
                    for i in range(len(pbox["centers"]) - 1):
                        if self.distance_3d(pbox["centers"][i], pbox["centers"][i + 1]) > dist_pending:
                            self.pending_boxes[digit].remove(pbox)
                            return
                    arr = np.array(pbox["centers"])
                    avg_center = tuple(arr.mean(axis=0))
                    self.finalized_boxes[digit].append(avg_center)
                    self.box_counts[digit] += 1
                    self.pending_boxes[digit].remove(pbox)
                return

        self.pending_boxes[digit].append({"centers": [box_center]})

    def ocr_process(self, img_rgb):
        return self.ocr_reader.readtext(img_rgb, allowlist='0123456789')

    def image_callback(self, img_msg):
        start_time = time.time()

        if self.lidar_points is None or self.camera_intrinsics is None:
            return

        self.update_transform(img_msg.header.stamp)
        if self.transform is None:
            return

        img = ros_numpy.numpify(img_msg)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        points = self.lidar_points.copy()
        lidar_cam_all = self.transform_points(points, self.transform)
        min_len = min(len(lidar_cam_all), len(points))
        lidar_cam = lidar_cam_all[:min_len]
        lidar_points = points[:min_len]

        valid_indices = lidar_cam[:, 2] > 0
        lidar_cam = lidar_cam[valid_indices]
        lidar_filtered = lidar_points[valid_indices]

        uv = (self.camera_intrinsics @ lidar_cam.T).T
        uv[:, :2] /= lidar_cam[:, 2:3]
        uv = uv[:, :2].astype(int)

        if time.time() - self.last_ocr_time > self.ocr_interval:
            self.last_ocr_time = time.time()
            future = self.executor.submit(self.ocr_process, img_rgb.copy())
            self.ocr_result = future

        results = self.ocr_result.result() if self.ocr_result and self.ocr_result.done() else []

        for bbox, text, conf in results:
            if conf > 0.5 and len(text.strip()) == 1 and text.strip() in "123456789":
                bbox = np.array(bbox, dtype=int)
                u_center = int(np.mean(bbox[:, 0]))
                v_center = int(np.mean(bbox[:, 1]))
                u_min, v_min = np.min(bbox[:, 0]), np.min(bbox[:, 1])
                u_max, v_max = np.max(bbox[:, 0]), np.max(bbox[:, 1])

                in_bbox_mask = (
                    (uv[:, 0] >= u_min) & (uv[:, 0] <= u_max) &
                    (uv[:, 1] >= v_min) & (uv[:, 1] <= v_max)
                )
                matched_points = lidar_filtered[in_bbox_mask]

                if len(matched_points) > self.match_points_num:
                    box_center_point = matched_points.mean(axis=0)
                    normal_vector = self.adjust_normal_direction(
                        self.estimate_normal(matched_points), box_center_point)
                    box_center = box_center_point + normal_vector * 0.4

                    center_pt = PointStamped()
                    center_pt.header.frame_id = "velodyne"
                    center_pt.point.x, center_pt.point.y, center_pt.point.z = box_center

                    try:
                        transform_world = self.tf_buffer.lookup_transform("map", "velodyne", rospy.Time(0), rospy.Duration(1.0))
                        center_world = tf2_geometry_msgs.do_transform_point(center_pt, transform_world)
                        world_box_center = (
                            center_world.point.x,
                            center_world.point.y,
                            center_world.point.z
                        )
                        self.handle_detection(text, world_box_center, self.dist_existing, self.dist_pending, self.required_frames)
                    except Exception as e:
                        rospy.logerr(f"World transform failed: {str(e)}")

        annotated_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        annotated_msg = ros_numpy.msgify(Image, annotated_bgr, encoding="bgr8")
        annotated_msg.header = img_msg.header
        self.image_pub.publish(annotated_msg)

        compressed_msg = CompressedImage()
        compressed_msg.header = img_msg.header
        compressed_msg.format = "jpeg"
        compressed_msg.data = np.array(
            cv2.imencode('.jpg', annotated_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 50])[1]
        ).tobytes()
        self.compressed_image_pub.publish(compressed_msg)

        if self.box_counts:
            min_digit = min(self.box_counts, key=self.box_counts.get)
            self.min_box_pub.publish(int(min_digit))

        end_time = time.time()
        rospy.loginfo(f"Processing time: {end_time - start_time:.2f} seconds")

if __name__ == "__main__":
    node = BoxCountNode()
    rospy.spin()
