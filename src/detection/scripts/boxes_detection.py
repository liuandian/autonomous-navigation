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
from geometry_msgs.msg import PointStamped, PoseStamped
from sklearn.decomposition import PCA
import time
import math
from collections import defaultdict
from std_msgs.msg import Int8, Bool

class BoxCountNode:
    def __init__(self):
        rospy.init_node("ocr_lidar_box_center_node")

        self.ocr_reader = easyocr.Reader(['en'], gpu=True)
        rospy.loginfo(f"EasyOCR initialized. Using GPU: {self.ocr_reader.device == 'cuda'}")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.lidar_points = None
        self.camera_intrinsics = None
        self.transform = None
        self.last_tf_update = rospy.Time(0)

        self.image_pub = rospy.Publisher("/detection/image_annotated", Image, queue_size=1)
        self.compressed_image_pub = rospy.Publisher("/detection/image_annotated/compressed", CompressedImage, queue_size=1)
        self.min_box_pub = rospy.Publisher("/detection/min_box_count", Int8, queue_size=1)
        self.goal_pub = rospy.Publisher("/goal_from_digit", PoseStamped, queue_size=1)

        self.finalized_boxes = defaultdict(list)
        self.pending_boxes = defaultdict(list)
        self.box_counts = defaultdict(int)
        self.match_points_num = 20
        self.dist_existing = 1.2
        self.dist_pending = 0.3
        self.required_frames = 20

        self.find_goal_mode = False
        self.boxes_count_mode = False
        self.min_digit = None

        rospy.Subscriber("/front/camera_info", CameraInfo, self.camera_info_callback)
        rospy.Subscriber("/mid/points", PointCloud2, self.lidar_callback)
        rospy.Subscriber("/front/image_raw", Image, self.image_callback)
        rospy.Subscriber("/do_find_goal", Bool, self.find_goal_callback)
        rospy.Subscriber("/do_boxes_count", Bool, self.boxes_count_callback)

    def find_goal_callback(self, msg):
        self.find_goal_mode = msg.data
        if msg.data:
            rospy.loginfo("[FIND_GOAL] 模式开启")

    def boxes_count_callback(self, msg):
        self.boxes_count_mode = msg.data
        if msg.data:
            rospy.loginfo("[BOXES_COUNT] 模式开启")

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
                self.transform = self.tf_buffer.lookup_transform("front_camera_optical", "velodyne", rospy.Time(0), rospy.Duration(1.0))
                self.last_tf_update = now
            except Exception as e:
                rospy.logwarn(f"TF lookup failed: {str(e)}")
                self.transform = None

    def transform_points(self, points, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rot_mat = tf.transformations.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])[:3, :3]
        trans_vec = np.array([translation.x, translation.y, translation.z])
        transformed = points @ rot_mat.T + trans_vec
        return transformed[~np.isnan(transformed).any(axis=1)]

    def distance_3d(self, c1, c2):
        return np.linalg.norm(np.array(c1) - np.array(c2))

    def handle_detection(self, digit, box_center):
        rospy.loginfo(f"Current box counts: {self.box_counts.items()}")
        for fc in self.finalized_boxes[digit]:
            if self.distance_3d(fc, box_center) < self.dist_existing:
                return
        for pbox in self.pending_boxes[digit]:
            last_center = pbox["centers"][-1]
            if self.distance_3d(last_center, box_center) < self.dist_pending:
                pbox["centers"].append(box_center)
                if len(pbox["centers"]) >= self.required_frames:
                    if all(self.distance_3d(pbox["centers"][i], pbox["centers"][i+1]) < self.dist_pending for i in range(len(pbox["centers"])-1)):
                        avg_center = np.mean(pbox["centers"], axis=0)
                        self.finalized_boxes[digit].append(tuple(avg_center))
                        self.box_counts[digit] += 1
                        self.pending_boxes[digit].remove(pbox)
                        rospy.loginfo(f"[CONFIRMED] {digit} at {avg_center}, total: {self.box_counts[digit]}")
                return
        self.pending_boxes[digit].append({"centers": [box_center]})

    def image_callback(self, img_msg):
        start_time = time.time()

        if not (self.find_goal_mode or self.boxes_count_mode):
            return

        if self.lidar_points is None or self.camera_intrinsics is None:
            return

        self.update_transform(img_msg.header.stamp)
        if self.transform is None:
            return

        img = ros_numpy.numpify(img_msg)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        points = self.lidar_points.copy()
        lidar_cam_all = self.transform_points(points, self.transform)
        lidar_cam = lidar_cam_all[lidar_cam_all[:, 2] > 0]
        lidar_filtered = points[lidar_cam_all[:, 2] > 0]
        uv = (self.camera_intrinsics @ lidar_cam.T).T
        uv[:, :2] /= lidar_cam[:, 2:3]
        uv = uv[:, :2].astype(int)

        results = self.ocr_reader.readtext(img_rgb, allowlist='0123456789')

        for bbox, text, conf in results:
            if conf > 0.5 and text.strip() in "123456789":
                bbox = np.array(bbox, dtype=int)
                u_center, v_center = int(np.mean(bbox[:, 0])), int(np.mean(bbox[:, 1]))
                u_min, v_min = np.min(bbox[:, 0]), np.min(bbox[:, 1])
                u_max, v_max = np.max(bbox[:, 0]), np.max(bbox[:, 1])
                in_bbox_mask = (
                    (uv[:, 0] >= u_min) & (uv[:, 0] <= u_max) &
                    (uv[:, 1] >= v_min) & (uv[:, 1] <= v_max))
                matched_points = lidar_filtered[in_bbox_mask]

                if len(matched_points) > self.match_points_num:
                    box_center_point = matched_points.mean(axis=0)
                    normal_vector = self.adjust_normal_direction(self.estimate_normal(matched_points), box_center_point)
                    box_center = box_center_point + normal_vector * 0.4
                    center_pt = PointStamped()
                    center_pt.header.frame_id = "velodyne"
                    center_pt.point.x, center_pt.point.y, center_pt.point.z = box_center

                    try:
                        transform_world = self.tf_buffer.lookup_transform("map", "velodyne", rospy.Time(0), rospy.Duration(1.0))
                        center_world = tf2_geometry_msgs.do_transform_point(center_pt, transform_world)
                        world_box_center = (center_world.point.x, center_world.point.y, center_world.point.z)

                        if self.find_goal_mode:
                            if self.min_digit is None and self.box_counts:
                                self.min_digit = min(self.box_counts, key=self.box_counts.get)
                            if text == str(self.min_digit):
                                goal = PoseStamped()
                                goal.header = center_world.header
                                goal.pose.position = center_world.point
                                goal.pose.orientation.w = 1.0
                                self.goal_pub.publish(goal)
                                rospy.loginfo(f"[GOAL] 识别到目标数字 {text}，已发送目标点: {goal.pose.position}")
                        elif self.boxes_count_mode:
                            self.handle_detection(text, world_box_center)
                    except Exception as e:
                        rospy.logwarn(f"Transform or handle failed: {e}")

        annotated_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        annotated_msg = ros_numpy.msgify(Image, annotated_bgr, encoding="bgr8")
        annotated_msg.header = img_msg.header
        self.image_pub.publish(annotated_msg)

        compressed_msg = CompressedImage()
        compressed_msg.header = img_msg.header
        compressed_msg.format = "jpeg"
        compressed_msg.data = np.array(cv2.imencode('.jpg', annotated_bgr)[1]).tobytes()
        self.compressed_image_pub.publish(compressed_msg)

        if self.box_counts:
            min_digit = min(self.box_counts, key=self.box_counts.get)
            self.min_box_pub.publish(int(min_digit))
            rospy.loginfo(f"[COUNT] min_digit = {min_digit}")

        rospy.loginfo(f"Processing time: {time.time() - start_time:.2f} seconds")

if __name__ == "__main__":
    node = BoxCountNode()
    rospy.spin()
