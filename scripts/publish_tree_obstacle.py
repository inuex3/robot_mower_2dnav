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
        #self.marker_publisher = rospy.Publisher("/detected_tree", Marker, queue_size = 1)
        self.now = rospy.get_rostime()
        self.prev = rospy.get_rostime()


    def make_msg(self, Image, detection_data, camera_param):
        self.now = rospy.get_rostime()
        self.timebefore = detection_data.header.stamp
        # Add point obstacle
        self.obstacle_msg = ObstacleArrayMsg() 
        self.obstacle_msg.header.stamp = rospy.Time.now()
        self.obstacle_msg.header.frame_id = Image.header.frame_id # CHANGE HERE: odom/map
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
                    tan_angle_x = camera_param[0][0]*(bbox.xmin+bbox.xmax)/2+camera_param[0][1]**(bbox.ymin+bbox.ymax)/2+camera_param[0][2]*1
                    distance_y = - distance_x * tan_angle_x
                    angle_x = math.atan(tan_angle_x)  
                    # Add point obstacle
                    self.obstacle_msg.obstacles.append(ObstacleMsg())
                    self.obstacle_msg.obstacles[i].id = i
                    self.obstacle_msg.obstacles[i].polygon.points = [Point32()]
                    self.obstacle_msg.obstacles[i].polygon.points[0].x = -distance_y
                    self.obstacle_msg.obstacles[i].polygon.points[0].y = 0
                    self.obstacle_msg.obstacles[i].polygon.points[0].z = distance_x
                    self.obstacle_msg.obstacles[i].radius = 0.15
                    self.marker_data.markers.append(Marker())
                    self.marker_data.markers[i].header.stamp, self.marker_data.markers[i].header.frame_id = rospy.Time.now(), Image.header.frame_id     
                    self.marker_data.markers[i].ns, self.marker_data.markers[i].id = "basic_shapes", i
                    #print(tf.transformations.quaternion_from_euler(-math.pi/2, 0, -math.pi/2))
                    print(tf.transformations.quaternion_from_euler(math.pi/2, math.pi/2, -math.pi/2))
                    self.marker_data.markers[i].action = Marker.ADD
                    self.marker_data.markers[i].pose.position.x, self.marker_data.markers[i].pose.position.y, self.marker_data.markers[i].pose.position.z = -distance_y, 0,distance_x
                    #self.marker_data.markers[i].pose.orientation.x, self.marker_data.markers[i].pose.orientation.y, self.marker_data.markers[i].pose.orientation.z, self.marker_data.markers[i].pose.orientation.w= -0.5, -0.5, -0.5, 0.5
                    #For Tree
                    #self.marker_data.markers[i].pose.orientation.x, self.marker_data.markers[i].pose.orientation.y, self.marker_data.markers[i].pose.orientation.z, self.marker_data.markers[i].pose.orientation.w= 0.5, -0.5, 0.5, 0.5
                    #For Person
                    self.marker_data.markers[i].pose.orientation.x, self.marker_data.markers[i].pose.orientation.y, self.marker_data.markers[i].pose.orientation.z, self.marker_data.markers[i].pose.orientation.w= 0.7071, 0, -0.7071, 0
                    self.marker_data.markers[i].color.r, self.marker_data.markers[i].color.g, self.marker_data.markers[i].color.b, self.marker_data.markers[i].color.a = 0.5, 0.7, 0, 1.0
                    self.marker_data.markers[i].scale.x, self.marker_data.markers[i].scale.y, self.marker_data.markers[i].scale.z = 0.2, 0.2, 0.2
                    self.marker_data.markers[i].type = 10
                    #self.marker_data.markers[i].mesh_resource = "package://robot_mower_2dnav/stl/Tree1.stl"                    
                    #self.marker_data.markers[i].mesh_resource = "package://robot_mower_2dnav/stl/hades.stl"
                    self.marker_data.markers[i].mesh_resource = "package://robot_mower_2dnav/stl/animated_walking_man.mesh"
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

            elif bbox.Class == "marker":
                self.reset_pose(0,0,0,0,0,0)
 
    def send_msg(self):
        # messageを送
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
        print (camera_parameter_org)
        self.camera_param_subscriber.unregister()

    def bounding_boxes_callback(self, depth_image, detection_data):
        self.pub.make_msg(depth_image,detection_data, self.camera_parameter)
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