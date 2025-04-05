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

rospy.init_node("ocr_lidar_box_center_node")

ocr_reader = easyocr.Reader(['en'], gpu=False)
rospy.loginfo("EasyOCR initialized. Using GPU: {}".format(ocr_reader.device == 'cuda'))
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

lidar_points = None
camera_intrinsics = None

image_pub = rospy.Publisher("/detection/image_annotated", Image, queue_size=1)

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

def image_callback(img_msg):
    global lidar_points, camera_intrinsics

    if lidar_points is None or camera_intrinsics is None:
        rospy.logwarn("Waiting for LiDAR points or camera intrinsics...")
        return

    img = ros_numpy.numpify(img_msg)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    try:
        transform = tf_buffer.lookup_transform("front_camera_optical", "velodyne", rospy.Time(0), rospy.Duration(1.0))
        #transform_world = tf_buffer.lookup_transform("base_link", "velodyne", rospy.Time(0), rospy.Duration(1.0))
    except Exception as e:
        rospy.logerr(f"TF transform lookup failed: {str(e)}")
        return

    results = ocr_reader.readtext(img_rgb,allowlist='0123456789')
    for bbox, text, conf in results:
        if conf > 0.5 and text.strip() in "123456789":
            rospy.loginfo(f"Detected text: {text} with confidence: {conf}")
            bbox = np.array(bbox, dtype=int)
            u_center = int(np.mean(bbox[:, 0]))
            v_center = int(np.mean(bbox[:, 1]))
            u_min, v_min = np.min(bbox[:, 0]), np.min(bbox[:, 1])
            u_max, v_max = np.max(bbox[:, 0]), np.max(bbox[:, 1])



            matched_points =[]
            for pt in lidar_points:
                pt_stamped = PointStamped()
                pt_stamped.header.frame_id = "velodyne"
                pt_stamped.point.x, pt_stamped.point.y, pt_stamped.point.z = pt.tolist()
                try:
                    pt_transformed = tf2_geometry_msgs.do_transform_point(pt_stamped, transform)
                    # Check if the point is in front of the camera
                    if pt_transformed.point.z <= 0:
                        continue
                    x, y, z = pt_transformed.point.x, pt_transformed.point.y, pt_transformed.point.z
                    uv = camera_intrinsics @ np.array([x, y, z]) / z

                    # Check if the point is within the bounding box
                    u_p, v_p = int(uv[0]), int(uv[1])
                    if u_min <= u_p <= u_max and v_min <= v_p <= v_max:
                        matched_points.append(pt)  
                        # Draw the point on the image
                        cv2.circle(img_rgb, (u_p, v_p), 3, (255, 0, 0), -1)  # Green
                except Exception as e:
                    rospy.logerr(f"Point transform failed: {str(e)}")
                    continue

            if np.count_nonzero(matched_points) > 15:
                matched_points = np.array(matched_points)
                box_center_point = np.mean(matched_points, axis=0)
                normal_vector = estimate_normal(matched_points)
                normal_vector = adjust_normal_direction(normal_vector, box_center_point)
                # box 0.8*0.8*0.8
                box_center = box_center_point + normal_vector * 0.4

                center_pt = PointStamped()
                center_pt.header.frame_id = "velodyne"
                center_pt.point.x, center_pt.point.y, center_pt.point.z = box_center.tolist()

                try:
                    # Transform the center point to the world frame
                    transform_world = tf_buffer.lookup_transform("map", "velodyne", rospy.Time(0), rospy.Duration(1.0))
                    center_world = tf2_geometry_msgs.do_transform_point(center_pt, transform_world)
                    rospy.loginfo(f"Digit [{text}] Box center at world: ({center_world.point.x:.2f}, {center_world.point.y:.2f}, {center_world.point.z:.2f})")
                    
                    #visualize the box center in camera frame
                    center_camera = tf2_geometry_msgs.do_transform_point(center_pt, transform)
                    x, y, z = center_camera.point.x, center_camera.point.y, center_camera.point.z
                    # Check if the point is in front of the camera
                    if z > 0:  
                        uv = camera_intrinsics @ np.array([x, y, z]) / z
                        u, v = int(uv[0]), int(uv[1])
                               
                    # Draw the real box center point
                    cv2.circle(img_rgb, (u, v), 5, (0, 255, 255), -1)  # rad
                    cv2.putText(img_rgb, f"{text}@({center_world.point.x:.2f},{center_world.point.y:.2f})", (u, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                    
                    # Draw the box
                    cv2.rectangle(img_rgb, tuple(bbox[0]), tuple(bbox[2]), (0, 255, 0), 2)
                    # cv2.putText(img_rgb, f"{text}@({center_world.point.x:.2f},{center_world.point.y:.2f})",
                    #             (u_center, v_center - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                    
                    # Draw the num center point
                    cv2.circle(img_rgb, (u_center, v_center), 5, (255, 255, 255), -1)  # white
            #         cv2.putText(img_rgb, f"({u_center}, {v_center})", (u_center + 10, v_center - 10),
            # cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)  
                except Exception as e:
                    rospy.logerr(f"World transform failed: {str(e)}")
            else:
                rospy.logwarn("Not enough inliers for box estimation")

    # Convert and publish annotated image
    annotated_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    annotated_msg = ros_numpy.msgify(Image, annotated_bgr, encoding="bgr8")
    annotated_msg.header = img_msg.header
    image_pub.publish(annotated_msg)

rospy.Subscriber("/front/image_raw", Image, image_callback)
rospy.spin()
