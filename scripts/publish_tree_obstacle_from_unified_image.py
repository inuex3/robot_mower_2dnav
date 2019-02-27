#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import message_filters
from mavros_msgs.msg import OverrideRCIn
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
        self.tf_br = tf.TransformBroadcaster()
        self.bboxes_from_camera1 = BoundingBoxes()
        self.bboxes_from_camera2 = BoundingBoxes()
        self.now = rospy.get_rostime()
        self.prev = rospy.get_rostime()


    def make_msg(self, Depth1Image, Depth2Image, detection_data, camera1_param, camera2s_param):
        self.bboxes_from_camera1 = BoundingBoxes()
        self.bboxes_from_camera2 = BoundingBoxes()
        self.bboxes_from_camera1.header = Depth1Image.header
        self.bboxes_from_camera2.header = Depth2Image.header
        self.now = rospy.get_rostime()
        self.timebefore = detection_data.header.stamp
        # Add point obstacle
        self.obstacle_msg = ObstacleArrayMsg() 
        self.obstacle_msg.header.stamp = rospy.Time.now()
        self.obstacle_msg.header.frame_id = "odom" # CHANGE HERE: odom/map
        self.marker_data = MarkerArray()
        self.rate = rospy.Rate(25)        
        #opencvに変換
        bridge = CvBridge()
        try:
            self.Depth1image = bridge.imgmsg_to_cv2(Depth1Image, desired_encoding="passthrough")
            self.Depth2image = bridge.imgmsg_to_cv2(Depth2Image, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)
        i = 0
        for bbox in detection_data.bounding_boxes:
            if (bbox.ymin + bbox.ymax) < self.Depth1image.shape[0]:
                if bbox.ymax > self.Depth1image.shape[0]:
                    bbox.ymax = self.Depth1image.shape[0]
                self.bboxes_from_camera1.bounding_boxes.append(bbox)
            else:
                bbox.ymin = bbox.ymin - self.Depth1image.shape[0]
                bbox.ymax = bbox.ymax - self.Depth1image.shape[0]
                if bbox.ymin < 0:
                    bbox.ymin = 0
                    print(bbox)
                self.bboxes_from_camera2.bounding_boxes.append(bbox)
            if bbox.Class == "person":
                try:
                    self.detected_area = self.Depth1image[bbox.ymin:bbox.ymax,  bbox.xmin:bbox.xmax]
                    distance_x = np.median(self.detected_area)/1000
                    tan_angle_x = camera_param[0][0]*(bbox.xmin+bbox.xmax)/2+camera_param[0][1]*(bbox.ymin+bbox.ymax)/2+camera_param[0][2]*1
                    distance_y = - distance_x * tan_angle_x
                    angle_x = math.atan(tan_angle_x) 
                    if abs(math.degrees(angle_x)) < 30:
                        self.tf_br.sendTransform((distance_x, distance_y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     Image.header.frame_id,
                     "Tree" + str(i))
                        # Add point obstacle
                        self.obstacle_msg.obstacles.append(ObstacleMsg())
                        self.obstacle_msg.obstacles[i].header.stamp, self.obstacle_msg.obstacles[i].header.frame_id = rospy.Time.now(), Image.header.frame_id     
                        self.obstacle_msg.obstacles[i].id = i
                        self.obstacle_msg.obstacles[i].polygon.points = [Point32()]
                        self.obstacle_msg.obstacles[i].polygon.points[0].x = -distance_y
                        self.obstacle_msg.obstacles[i].polygon.points[0].y = 0
                        self.obstacle_msg.obstacles[i].polygon.points[0].z = distance_x
                        self.obstacle_msg.obstacles[i].radius = 0.15
                        self.marker_data.markers.append(Marker())
                        self.marker_data.markers[i].header.stamp, self.marker_data.markers[i].header.frame_id = rospy.Time.now(), Image.header.frame_id     
                        self.marker_data.markers[i].ns, self.marker_data.markers[i].id = "basic_shapes", i
                        self.marker_data.markers[i].action = Marker.ADD
                        self.marker_data.markers[i].pose.position.x, self.marker_data.markers[i].pose.position.y, self.marker_data.markers[i].pose.position.z = -distance_y, 0,distance_x
                        self.marker_data.markers[i].pose.orientation.x, self.marker_data.markers[i].pose.orientation.y, self.marker_data.markers[i].pose.orientation.z, self.marker_data.markers[i].pose.orientation.w= -0.5, -0.5, -0.5, 0.5
                        self.marker_data.markers[i].color.r, self.marker_data.markers[i].color.g, self.marker_data.markers[i].color.b, self.marker_data.markers[i].color.a = 1, 0, 0, 1.0
                        self.marker_data.markers[i].scale.x, self.marker_data.markers[i].scale.y, self.marker_data.markers[i].scale.z = 0.2, 0.2, 1
                        self.marker_data.markers[i].type = 3
                        self.marker_data.markers[i].lifetime = rospy.Duration.from_sec(2.0)
                        i += 1
                except CvBridgeError as e:
                        pass

            elif bbox.Class == "marker":
                self.reset_pose(0,0,0,0,0,0)
        
    def bbox_to_obstacle(bboxes, DepthImage, camera_param):
        for bbox in bboxes:
            if bbox.Class == "person":
                try:
                    detected_area = Depthimage[bbox.ymin:bbox.ymax,  bbox.xmin:bbox.xmax]
                    distance_x = np.median(detected_area)/1000
                    tan_angle_x = camera_param[0][0]*(bbox.xmin+bbox.xmax)/2+camera_param[0][1]*(bbox.ymin+bbox.ymax)/2+camera_param[0][2]*1
                    distance_y = - distance_x * tan_angle_x
                    angle_x = math.atan(tan_angle_x) 
                    if abs(math.degrees(angle_x)) < 30:
                        self.tf_br.sendTransform((distance_x, distance_y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     bboxes.header.stamp,
                     DepthImage.header.frame_id,
                     bbox.Class + str(i))
                        obstable = tf.lookupTransform("odom", bbox.Class + str(i), bboxes.header.stamp)
 
    def send_msg(self):
        self.publisher.publish(self.obstacle_msg)
        self.marker_publisher.publish(self.marker_data)

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
