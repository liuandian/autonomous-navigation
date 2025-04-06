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
from collections import defaultdict
from std_msgs.msg import Int8
# 关键：引入 message_filters
import message_filters

import threading

class BoxCountNode:
    def __init__(self):
        #rospy.init_node("ocr_lidar_box_center_node")
        rospy.init_node("ocr_lidar_box_center_node", disable_signals=True)
        self.ocr_reader = easyocr.Reader(['en'], gpu=True)
        rospy.loginfo(f"EasyOCR initialized. Using GPU: {self.ocr_reader.device == 'cuda'}")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.camera_intrinsics = None
        self.transform = None
        self.last_tf_update = rospy.Time(0)

        self.image_pub = rospy.Publisher("/detection/image_annotated", Image, queue_size=1)
        self.compressed_image_pub = rospy.Publisher("/detection/image_annotated/compressed", CompressedImage, queue_size=1)
        self.min_box_pub = rospy.Publisher("/detection/min_box_count", Int8, queue_size=1)

        # 存放已确认的 box（digit -> [(x, y, z), ...]）
        self.finalized_boxes = defaultdict(list)
        # 存放待确认 box（digit -> [ { "centers": [(x, y, z), ...] }, ... ]）
        self.pending_boxes = defaultdict(list)
        # 每个 digit 已确认 box 数
        self.box_counts = defaultdict(int)
        self.match_points_num = 20
        self.dist_existing = 1.2
        self.dist_pending = 0.3
        self.required_frames = 20

        # 订阅相机内参（通常可以不用时间同步）
        rospy.Subscriber("/front/camera_info", CameraInfo, self.camera_info_callback)
        rospy.loginfo(f"test!!!!!!!!!!!")
        image_sub = message_filters.Subscriber("/front/image_raw", Image)
        lidar_sub = message_filters.Subscriber("/mid/points", PointCloud2)

        # 创建近似时间同步器
        sync = message_filters.ApproximateTimeSynchronizer(
            [image_sub, lidar_sub],
            queue_size=50,  # 队列大小
            slop=0.2       # 允许的时间差（单位：秒）
        )
        sync.registerCallback(self.synced_callback)
        self.start_time = time.time()
    def camera_info_callback(self, info):
        # 获取相机内参
        self.camera_intrinsics = np.array(info.K).reshape(3, 3)
        self.end_time = time.time()
        rospy.loginfo(f"Time elapsed: {self.end_time - self.start_time:.2f} seconds")
        self.start_time = self.end_time

    def synced_callback(self, img_msg, pc2_msg):

        diff = abs((img_msg.header.stamp - pc2_msg.header.stamp).to_sec())
        rospy.loginfo(f"Image ts: {img_msg.header.stamp.to_sec():.3f}, Lidar ts: {pc2_msg.header.stamp.to_sec():.3f}, Δt: {diff:.3f}")
        return
        """
        同步回调函数，同时接收图像与点云消息。
        原先 image_callback + lidar_callback 的逻辑合并到这里。
        """
        start_time = time.time()

        if self.camera_intrinsics is None:
            rospy.logwarn("Waiting for camera intrinsics...")
            return

        self.update_transform(img_msg.header.stamp)
        if self.transform is None:
            return

        # 解析图像
        img = ros_numpy.numpify(img_msg)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # 解析点云
        points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_msg, remove_nans=True)

        # 转换点云到相机坐标
        lidar_cam_all = self.transform_points(points, self.transform)
        min_len = min(len(lidar_cam_all), len(points))
        lidar_cam = lidar_cam_all[:min_len]
        lidar_points = points[:min_len]

        # 只保留在相机前方的点（z>0）
        valid_indices = lidar_cam[:, 2] > 0
        lidar_cam = lidar_cam[valid_indices]
        lidar_filtered = lidar_points[valid_indices]

        # 将点云投影到图像平面
        uv = (self.camera_intrinsics @ lidar_cam.T).T
        uv[:, :2] /= lidar_cam[:, 2:3]
        uv = uv[:, :2].astype(int)

        # OCR 检测
        results = self.ocr_reader.readtext(img_rgb, allowlist='0123456789')
        for bbox, text, conf in results:
            if conf > 0.5 and len(text.strip()) == 1 and text.strip() in "123456789":
                bbox = np.array(bbox, dtype=int)
                u_center = int(np.mean(bbox[:, 0]))
                v_center = int(np.mean(bbox[:, 1]))
                u_min, v_min = np.min(bbox[:, 0]), np.min(bbox[:, 1])
                u_max, v_max = np.max(bbox[:, 0]), np.max(bbox[:, 1])

                in_bbox_mask = (
                    (uv[:, 0] >= u_min) &
                    (uv[:, 0] <= u_max) &
                    (uv[:, 1] >= v_min) &
                    (uv[:, 1] <= v_max)
                )
                matched_points = lidar_filtered[in_bbox_mask]
                for u_p, v_p in uv[in_bbox_mask]:
                    cv2.circle(img_rgb, (u_p, v_p), 3, (255, 0, 0), -1)

                if len(matched_points) > self.match_points_num:
                    box_center_point = matched_points.mean(axis=0)
                    normal_vector = self.adjust_normal_direction(
                        self.estimate_normal(matched_points),
                        box_center_point
                    )
                    box_center = box_center_point + normal_vector * 0.4

                    center_pt = PointStamped()
                    center_pt.header.frame_id = "velodyne"
                    center_pt.point.x, center_pt.point.y, center_pt.point.z = box_center

                    try:
                        # 转到 map 坐标系
                        transform_world = self.tf_buffer.lookup_transform(
                            "map", "velodyne",
                            rospy.Time(0), rospy.Duration(1.0)
                        )
                        center_world = tf2_geometry_msgs.do_transform_point(center_pt, transform_world)
                        world_box_center = (
                            center_world.point.x,
                            center_world.point.y,
                            center_world.point.z
                        )
                        # 处理检测到的数字箱子
                        self.handle_detection(
                            digit=text,
                            box_center=world_box_center,
                            dist_existing=self.dist_existing,
                            dist_pending=self.dist_pending,
                            required_frames=self.required_frames
                        )
                        # 在图像上标记
                        center_camera = tf2_geometry_msgs.do_transform_point(center_pt, self.transform)
                        x, y, z = center_camera.point.x, center_camera.point.y, center_camera.point.z
                        if z > 0:
                            uv_center = self.camera_intrinsics @ np.array([x, y, z]) / z
                            u, v = int(uv_center[0]), int(uv_center[1])
                            cv2.circle(img_rgb, (u, v), 5, (0, 255, 255), -1)
                            cv2.putText(
                                img_rgb,
                                f"{text} ({center_world.point.x:.2f},{center_world.point.y:.2f})",
                                (u, v - 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (255, 255, 0), 1
                            )
                            cv2.rectangle(img_rgb, tuple(bbox[0]), tuple(bbox[2]), (0, 255, 0), 2)
                            cv2.circle(img_rgb, (u_center, v_center), 5, (255, 255, 255), -1)
                    except Exception as e:
                        rospy.logerr(f"World transform failed: {str(e)}")
                else:
                    rospy.logwarn("Not enough inliers for box estimation")

        # 输出图像和压缩图像
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

        # 发布当前计数最少的 digit
        if self.box_counts:
            min_digit = min(self.box_counts, key=self.box_counts.get)
            self.min_box_pub.publish(int(min_digit))
            rospy.loginfo(f"min_digit {min_digit}")

        end_time = time.time()
        rospy.loginfo(f"Processing time: {end_time - start_time:.2f} seconds")

    def update_transform(self, data_time):
        now = rospy.Time.now()
        if (now - self.last_tf_update).to_sec() > 1.0:  # 每秒更新一次
            try:
                self.transform = self.tf_buffer.lookup_transform(
                    "front_camera_optical", "velodyne",
                    rospy.Time(0), rospy.Duration(1.0)
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

    def estimate_normal(self, points):
        pca = PCA(n_components=3)
        pca.fit(points)
        return pca.components_[-1]

    def adjust_normal_direction(self, normal_vector, box_center_point):
        if np.dot(normal_vector, box_center_point) < 0:
            normal_vector = -normal_vector
        return normal_vector

    def distance_3d(self, c1, c2):
        return math.sqrt((c1[0] - c2[0])**2 + (c1[1] - c2[1])**2 + (c1[2] - c2[2])**2)

    def handle_detection(self, digit, box_center, dist_existing=1.0, dist_pending=0.5, required_frames=4):
        rospy.loginfo(f"Current box counts: {self.box_counts.items()}")
        # 先检查已确认的 box
        for fc in self.finalized_boxes[digit]:
            if self.distance_3d(fc, box_center) < dist_existing:
                return

        # 若不在已确认 box 中，检查是否可合并到 pending box
        for pbox in self.pending_boxes[digit]:
            last_center = pbox["centers"][-1]
            if self.distance_3d(last_center, box_center) < dist_pending:
                pbox["centers"].append(box_center)
                if len(pbox["centers"]) >= required_frames:
                    for i in range(len(pbox["centers"]) - 1):
                        if self.distance_3d(pbox["centers"][i], pbox["centers"][i+1]) > dist_pending:
                            self.pending_boxes[digit].remove(pbox)
                            return
                    arr = np.array(pbox["centers"])
                    avg_center = tuple(arr.mean(axis=0))
                    self.finalized_boxes[digit].append(avg_center)
                    self.box_counts[digit] += 1
                    rospy.loginfo(
                        f"Confirmed new box for digit {digit} at {avg_center}, total={self.box_counts[digit]}"
                    )
                    self.pending_boxes[digit].remove(pbox)
                return

        # 若所有 pending box 都不合适，则新建一个
        self.pending_boxes[digit].append({"centers": [box_center]})

if __name__ == "__main__":
    node = BoxCountNode()
    #rospy.spin()
    spinner = rospy.AsyncSpinner(4)  # 使用4个线程
    spinner.start()
