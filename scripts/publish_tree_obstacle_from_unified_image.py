#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import message_filters
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PolygonStamped, Point32
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from rtabmap_ros.srv import ResetPose
import cv2
import math
import tf


class Publishsers():
    def __init__(self):
        # Publisherを作成
        self.publisher = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
        self.marker_publisher = rospy.Publisher("/detected_tree", MarkerArray, queue_size = 1)
        self.reset_pose = rospy.ServiceProxy('/rtabmap/reset_odom_to_pose', ResetPose)
        self.obstacle_list = ["person", "tree"]
        self.tf_br = tf.TransformBroadcaster()
        self.now = rospy.get_rostime()
        self.prev = rospy.get_rostime()

    def make_msg(self, Depth1Image, Depth2Image, detection_data, camera1_param, camera2s_param):
        bboxes_from_camera1 = BoundingBoxes()
        bboxes_from_camera2 = BoundingBoxes()
        bboxes_from_camera1.header = Depth1Image.header
        bboxes_from_camera2.header = Depth2Image.header
        self.now = rospy.get_rostime()
        self.timebefore = detection_data.header.stamp
        # Add point obstacle
        self.obstacle_msg = ObstacleArrayMsg() 
        self.obstacle_msg.header.stamp = detection_data.header.stamp
        self.obstacle_msg.header.frame_id = "odom" # CHANGE HERE: odom/map
        self.marker_data = MarkerArray()
        #opencvに変換
        bridge = CvBridge()
        try:
            Depth1image = bridge.imgmsg_to_cv2(Depth1Image, desired_encoding="passthrough")
            Depth2image = bridge.imgmsg_to_cv2(Depth2Image, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)
        for bbox in detection_data.bounding_boxes:
            if (bbox.ymin + bbox.ymax) < Depth1image.shape[0]:
                if bbox.ymax > Depth1image.shape[0]:
                    bbox.ymax = Depth1image.shape[0]
                bboxes_from_camera1.bounding_boxes.append(bbox)
            else:
                bbox.ymin = bbox.ymin - Depth1image.shape[0]
                bbox.ymax = bbox.ymax - Depth1image.shape[0]
                if bbox.ymin < 0:
                    bbox.ymin = 0
                bboxes_from_camera2.bounding_boxes.append(bbox)
        camera1_obstacle_msg, camera1_marker_data = self.bbox_to_position_in_odom(bboxes_from_camera1, Depth1image, camera1_param)
        camera2_obstacle_msg, camera2_marker_data = self.bbox_to_position_in_odom(bboxes_from_camera2, Depth2image, camera2_param)
        self.obstacle_msg.obstacles.append(camera1_obstacle_msg.obstacles)
        self.obstacle_msg.obstacles.append(camera2_obstacle_msg.obstacles)
        self.marker_data.markers.append(camera1_marker_data.markers)
        self.marker_data.markers.append(camera2_marker_data.markers)

    def send_msg(self):
        self.publisher.publish(self.obstacle_msg)
        self.marker_publisher.publish(self.marker_data)        
    
    def bbox_to_position_in_odom(self, bboxes, DepthImage, camera_param):
        obstacle_msg = ObstacleArrayMsg() 
        marker_data = MarkerArray()
        for i, bbox in enumerate(bboxes):
            if bbox.Class in self.obstacle_list:
                try:
                    tan_angle_x = camera_param[0][0]*(bbox.xmin+bbox.xmax)/2+camera_param[0][1]*(bbox.ymin+bbox.ymax)/2+camera_param[0][2]*1                    
                    angle_x = math.atan(tan_angle_x)
                    if abs(math.degrees(angle_x)) < 30:
                        detected_area = DepthImage[bbox.ymin:bbox.ymax,  bbox.xmin:bbox.xmax]
                        distance_x = np.median(detected_area)/1000
                        distance_y = - distance_x * tan_angle_x
                        if distance_x < 4:
                            obstacle_msg.obstacles.append(ObstacleMsg())
                            tf_br.sendTransform((-distance_y, 0, distance_x), tf.transformations.quaternion_from_euler(0, 0, 0), bboxes.header.stamp, DepthImage.header.frame_id, bbox.Class + str(i))
                            obstable_position = tf.lookupTransform("odom", bbox.Class + str(i), bboxes.header.stamp)
                            obstacle_msg.obstacles.append(ObstacleMsg())
                            obstacle_msg.obstacles[i].header.stamp, self.obstacle_msg.obstacles[i].header.frame_id = bboxes.header.stamp, "odom"    
                            obstacle_msg.obstacles[i].id = i
                            obstacle_msg.obstacles[i].polygon.points = [Point32()]
                            obstacle_msg.obstacles[i].polygon.points[0].x = obstable_position[0]
                            obstacle_msg.obstacles[i].polygon.points[0].y = obstable_position[1]
                            obstacle_msg.obstacles[i].polygon.points[0].z = 0
                            obstacle_msg.obstacles[i].radius = 0.15  
                            marker_data.markers.append(Marker())
                            marker_data.markers[i].header.stamp, marker_data.markers[i].header.frame_id = bboxes.header.stamp, "odom"     
                            marker_data.markers[i].ns, marker_data.markers[i].id = bbox.Class, i
                            marker_data.markers[i].action = Marker.ADD
                            marker_data.markers[i].pose.position.x, marker_data.markers[i].pose.position.y, marker_data.markers[i].pose.position.z = obstable_position[0], obstable_position[1], 0
                            marker_data.markers[i].pose.orientation.x, marker_data.markers[i].pose.orientation.y, marker_data.markers[i].pose.orientation.z, marker_data.markers[i].pose.orientation.w= -0.5, -0.5, -0.5, 0.5
                            marker_data.markers[i].color.r, marker_data.markers[i].color.g, marker_data.markers[i].color.b, marker_data.markers[i].color.a = 1, 0, 0, 1.0
                            marker_data.markers[i].scale.x, marker_data.markers[i].scale.y, marker_data.markers[i].scale.z = 0.2, 0.2, 1
                            marker_data.markers[i].type = 3
                            marker_data.markers[i].lifetime = rospy.Duration.from_sec(3.0)
                except Exception as e:
                    print(e)
        return obstacle_msg, marker_data


class Subscribe_publishers():
    def __init__(self, pub):
        # Subscriberを作成
        self.depth1_subscriber = message_filters.Subscriber('/camera1/aligned_depth_to_color/image_raw', Image)
        self.depth2_subscriber = message_filters.Subscriber('/camera2/aligned_depth_to_color/image_raw', Image)
        self.detection_subscriber =message_filters.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes)
        self.camera1_param_subscriber = rospy.Subscriber("/camera1/color/camera_info", CameraInfo, self.camera1_parameter_callback)
        self.camera2_param_subscriber = rospy.Subscriber("/camera2/color/camera_info", CameraInfo, self.camera2_parameter_callback)
        # messageの型を作成
        self.pub = pub
        self.camera1_parameter = CameraInfo()
        self.camera2_parameter = CameraInfo()
        ts = message_filters.ApproximateTimeSynchronizer([self.depth1_subscriber, self.depth2_subscriber, self.detection_subscriber], 10, 0.05)
        ts.registerCallback(self.bounding_boxes_callback)

    def camera1_parameter_callback(self, data):
        camera_parameter_org = data.K
        camera_parameter_org = np.reshape(camera_parameter_org, (3, 3))
        self.camera1_parameter = np.linalg.inv(camera_parameter_org)
        print ("Camera parameter:")
        print (camera_parameter_org)
        self.camera1_param_subscriber.unregister()

    def camera2_parameter_callback(self, data):
        camera_parameter_org = data.K
        camera_parameter_org = np.reshape(camera_parameter_org, (3, 3))
        self.camera2_parameter = np.linalg.inv(camera_parameter_org)
        print ("Camera parameter:")
        print (camera_parameter_org)
        self.camera2_param_subscriber.unregister()        

    def bounding_boxes_callback(self, depth1_image, depth2_image, detection_data):
        self.pub.make_msg(depth1_image, depth2_image, detection_data, self.camera1_parameter, self.camera2_parameter)
        self.pub.send_msg()

def main():
    # nodeの立ち上げ
    rospy.init_node('publish_tree')
    rospy.wait_for_service('/rtabmap/reset_odom_to_pose/')

    # クラスの作成
    pub = Publishsers()
    sub = Subscribe_publishers(pub)

    rospy.spin()

if __name__ == '__main__':
    main()
