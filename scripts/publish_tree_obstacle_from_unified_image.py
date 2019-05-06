#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import message_filters
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point32
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
import tf
import cv2
import math


class Publishsers():
    def __init__(self):
        # Publisherを作成
        self.publisher = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
        self.marker_publisher = rospy.Publisher("/visualized_obstacle", MarkerArray, queue_size = 1)
        self.obstacle_list = ["person"]
        self.tf_br = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.obstacle_msg = ObstacleArrayMsg() 
        self.marker_data = MarkerArray()
        self.now = rospy.get_rostime()
        self.prev = rospy.get_rostime()

    def make_msg(self, Depth1Image, Depth2Image, detection_data, camera1_param, camera2_param):
        bboxes_from_camera1 = BoundingBoxes()
        bboxes_from_camera2 = BoundingBoxes()
        bboxes_from_camera1.header = Depth1Image.header
        bboxes_from_camera2.header = Depth2Image.header
        self.now = rospy.get_rostime()
        self.timebefore = detection_data.header.stamp
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
            if (bbox.ymin + bbox.ymax)/2 < Depth1image.shape[0]:
                if bbox.ymax > Depth1image.shape[0]:
                    bbox.ymax = Depth1image.shape[0]
                bboxes_from_camera1.bounding_boxes.append(bbox)
            else:
                bbox.ymin = bbox.ymin - Depth1image.shape[0]
                bbox.ymax = bbox.ymax - Depth1image.shape[0]
                if bbox.ymin < 0:
                    bbox.ymin = 0
                bboxes_from_camera2.bounding_boxes.append(bbox)
        self.obstacle_msg = ObstacleArrayMsg()
        self.marker_data = MarkerArray()
        camera1_obstacle, camera1_marker = self.bbox_to_position_in_odom(bboxes_from_camera1, Depth1image, camera1_param, 1)
        camera2_obstacle, camera2_marker = self.bbox_to_position_in_odom(bboxes_from_camera2, Depth2image, camera2_param, 2)
        self.obstacle_msg.obstacles.append(camera1_obstacle)
        self.marker_data.markers.append(camera1_marker)
        self.obstacle_msg.obstacles.append(camera2_obstacle)
        self.marker_data.markers.append(camera2_marker)
        
    def send_msg(self):
        self.publisher.publish(self.obstacle_msg)
        self.marker_publisher.publish(self.marker_data)      
    
    def bbox_to_position_in_odom(self, bboxes, DepthImage, camera_param, i):
        self.tf_listener.waitForTransform("/base_link", bbox.Class + str(i), rospy.Time(0), rospy.Duration(0.1))
        distance_to_obstacle = self.tf_listener.lookupTransform("/base_link", bbox.Class + str(i),  rospy.Time(0))
        prev_dist = distance_to_obstacle[0][0]*obstable_position[0][0] + obstable_position[0][1]*obstable_position[0][1]
        obstacle = ObstacleMsg()
        marker = Marker()
        for bbox in bboxes.bounding_boxes:
            if bbox.Class in self.obstacle_list:
                try:
                    tan_angle_x = camera_param[0][0]*(bbox.xmin+bbox.xmax)/2+camera_param[0][1]*(bbox.ymin+bbox.ymax)/2+camera_param[0][2]*1 
                    angle_x = math.atan(tan_angle_x)
                    if abs(math.degrees(angle_x)) < 40:
                        detected_area = DepthImage[bbox.ymin:bbox.ymax,  bbox.xmin:bbox.xmax]
                        distance_x = np.median(detected_area) / 1000
                        distance_x = distance_x + 0.15
                        distance_y = - distance_x * tan_angle_x
                        if 1.0 < distance_x and distance_x < prev_dist:
                            self.tf_br.sendTransform((-distance_y, 0, distance_x), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), bbox.Class + str(i), bboxes.header.frame_id)
                            self.tf_listener.waitForTransform("/odom", "/" + bbox.Class + str(i), rospy.Time(0), rospy.Duration(0.3))
                            obstable_position = self.tf_listener.lookupTransform("/odom", bbox.Class + str(i),  rospy.Time(0))
                            obstacle.header.stamp, obstacle.header.frame_id = bboxes.header.stamp, "odom"    
                            obstacle.id = i
                            obstacle.polygon.points = [Point32()]
                            obstacle.polygon.points[0].x = obstable_position[0][0]
                            obstacle.polygon.points[0].y = obstable_position[0][1]
                            obstacle.polygon.points[0].z = obstable_position[0][2]
                            marker.header.stamp, marker.header.frame_id = bboxes.header.stamp, "odom"     
                            marker.ns, marker.id = bbox.Class, i
                            marker.action = Marker.ADD
                            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = obstable_position[0][0], obstable_position[0][1], obstable_position[0][2]
                            marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w= tf.transformations.quaternion_from_euler(0, 0, 0)
                            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1, 0, 0, 1
                            marker.scale.x, marker.scale.y, marker.scale.z = 0.2, 0.2, 1
                            marker.type = 3
                            prev_dist = distance_x * distance_x + distance_y * distance_y
                except Exception as e:
                    print(e)
        return obstacle, marker

class Subscribe_publishers():
    def __init__(self, pub):
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

    # クラスの作成
    pub = Publishsers()
    sub = Subscribe_publishers(pub)

    rospy.spin()

if __name__ == '__main__':
    main()
