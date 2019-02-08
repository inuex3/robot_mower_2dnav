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
import cv2
import math


class Publishsers():
    def __init__(self):
        # Publisherを作成
        self.publisher = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
        self.marker_publisher = rospy.Publisher("/detected_tree", MarkerArray, queue_size = 1)
        #self.marker_publisher = rospy.Publisher("/detected_tree", Marker, queue_size = 1)
        self.now = rospy.get_rostime()
        self.prev = rospy.get_rostime()


    def make_msg(self, Image, detection_data, camera_param):
        self.now = rospy.get_rostime()
        self.timebefore = detection_data.header.stamp
        # Add point obstacle
        self.obstacle_msg = ObstacleArrayMsg() 
        self.obstacle_msg.header.stamp = rospy.Time.now()
        self.obstacle_msg.header.frame_id = "camera_link" # CHANGE HERE: odom/map
        #self.marker_data = Marker()
        self.marker_data = MarkerArray()
        self.rate = rospy.Rate(25)        
        #opencvに変換
        bridge = CvBridge()
        try:
            self.image = bridge.imgmsg_to_cv2(Image, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)
        i = 0
        for bbox in detection_data.bounding_boxes:
            if bbox.Class == "person":
                try:
                    self.detected_area = self.image[bbox.ymin:bbox.ymax,  bbox.xmin:bbox.xmax]
                    distance_x = np.median(self.detected_area)/1000
                    distance_y = - distance_x * (camera_param[0][0]*(bbox.xmin+bbox.xmax)/2+camera_param[0][1]**(bbox.ymin+bbox.ymax)/2+camera_param[0][2]*1)
                    angle_x = math.atan(camera_param[0][0]*(bbox.xmin+bbox.xmax)/2+camera_param[0][1]**(bbox.ymin+bbox.ymax)/2+camera_param[0][2]*1)  
                    # Add point obstacle
                    self.obstacle_msg.obstacles.append(ObstacleMsg())
                    self.obstacle_msg.obstacles[i].id = i
                    self.obstacle_msg.obstacles[i].polygon.points = [Point32()]
                    self.obstacle_msg.obstacles[i].polygon.points[0].x = distance_x
                    self.obstacle_msg.obstacles[i].polygon.points[0].y = distance_y
                    self.obstacle_msg.obstacles[i].polygon.points[0].z = 0
                    self.obstacle_msg.obstacles[i].radius = 0.15
                    self.marker_data.markers.append(Marker())
                    self.marker_data.markers[i].header.stamp = rospy.Time.now()        
                    self.marker_data.markers[i].header.frame_id = "map"
                    self.marker_data.markers[i].ns = "basic_shapes"
                    self.marker_data.markers[i].id = i
                    print(i)
                    self.marker_data.markers[i].action = Marker.ADD
                    self.marker_data.markers[i].pose.position.x = distance_x
                    self.marker_data.markers[i].pose.position.y = distance_y
                    self.marker_data.markers[i].pose.position.z = 0.0
                    self.marker_data.markers[i].pose.orientation.x=0.0
                    self.marker_data.markers[i].pose.orientation.y=0.0
                    self.marker_data.markers[i].pose.orientation.z=1.0
                    self.marker_data.markers[i].pose.orientation.w=0.0
                    self.marker_data.markers[i].color.r = 1.0
                    self.marker_data.markers[i].color.g = 0.0
                    self.marker_data.markers[i].color.b = 0.0
                    self.marker_data.markers[i].color.a = 1.0
                    self.marker_data.markers[i].scale.x = 0.15
                    self.marker_data.markers[i].scale.y = 0.15
                    self.marker_data.markers[i].scale.z = 1
                    self.marker_data.markers[i].lifetime = self.now - self.prev
                    self.marker_data.markers[i].type = 3
                    """
                    self.marker_data.action = Marker.ADD
                    self.marker_data.pose.position.x = distance_x
                    self.marker_data.pose.position.y = distance_y
                    self.marker_data.pose.position.z = 0.0
                    self.marker_data.pose.orientation.x=0.0
                    self.marker_data.pose.orientation.y=0.0
                    self.marker_data.pose.orientation.z=1.0
                    self.marker_data.pose.orientation.w=0.0
                    self.marker_data.color.r = 1.0
                    self.marker_data.color.g = 0.0
                    self.marker_data.color.b = 0.0
                    self.marker_data.color.a = 1.0
                    self.marker_data.scale.x = 1
                    self.marker_data.scale.y = 1
                    self.marker_data.scale.z = 1
                    """
                    rate = rospy.Rate(7)
                    #angle_y = arctan(camera_param[1][0]*center_x+camera_param[1][1]*center_y+camera_param[1][2]*1)
                    #self.detected_area.flags.writeable = True
                    #self.detected_area = np.where(self.detected_area < np.percentile(self.detected_area[np.nonzero(self.detected_area)], 40), 0, self.detected_area)
                    #self.detected_area = np.where(self.detected_area > np.percentile(self.detected_area[np.nonzero(self.detected_area)], 60), 0, self.detected_area)
                    #self.detected_area = np.where(self.detected_area < np.median(self.detected_area), 0, self.detected_area)
                    #print("distance_x" + str(distance_x) + "distance_y" + str(distance_y) + "angle_x" + str(math.degrees(angle_x)))
                    #print("difference" + str(np.median(self.detected_area.nonzero())-np.median(self.detected_area)))
                    i += 1
                except CvBridgeError as e:
                        pass
        #print("time difference" + str(Image.header.stamp - detection_data.header.stamp))
        #print(rospy.Time.now()-self.timebefore)
        self.prev = self.now
        

    def send_msg(self):
        # messageを送
        print("OK")
        self.publisher.publish(self.obstacle_msg)
        self.marker_publisher.publish(self.marker_data)

class Subscribe_publishers():
    def __init__(self, pub):
        # Subscriberを作成
        self.depth_subscriber = message_filters.Subscriber('/camera1/aligned_depth_to_color/image_raw', Image)
        self.detection_subscriber =message_filters.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes)
        self.camera_param_subscriber = rospy.Subscriber("/camera1/color/camera_info", CameraInfo, self.camera_parameter_callback)
        # messageの型を作成
        self.pub = pub
        self.camera_parameter = CameraInfo()
        ts = message_filters.ApproximateTimeSynchronizer([self.depth_subscriber, self.detection_subscriber], 10, 0.1)
        ts.registerCallback(self.bounding_boxes_callback)
        print("ready")

    def camera_parameter_callback(self, data):
        camera_parameter_org = data.K
        camera_parameter_org = np.reshape(camera_parameter_org, (3, 3))
        self.camera_parameter = np.linalg.inv(camera_parameter_org)
        print ("Camera parameter:")
        print (self.camera_parameter)
        self.camera_param_subscriber.unregister()

    def bounding_boxes_callback(self, depth_image, detection_data):
        self.pub.make_msg(depth_image,detection_data, self.camera_parameter)
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
