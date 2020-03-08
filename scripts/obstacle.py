#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import message_filters
from sensor_msgs.msg import Image,CameraInfo, NavSatFix
from cv_bridge import CvBridge, CvBridgeError
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point32
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from nav_msgs.msg import Odometry
#from apriltags2_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
import tf
import cv2
import math


class Publishsers():
    def __init__(self):
        # Publisherを作成
        self.publisher = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
        self.marker_publisher = rospy.Publisher("/visualized_obstacle", MarkerArray, queue_size = 1)
        #self.landmark_publisher = rospy.Publisher("/rtabmap/tag_detections", AprilTagDetectionArray, queue_size = 1)
        self.gnss_publisher = rospy.Publisher("/rtabmap/global_pose", PoseWithCovarianceStamped, queue_size = 1)
        self.object_list = ["landmark", "obstacle"]
        self.obstacle_list = ["obstacle", "landmark"]
        self.landmark_list = ["landmark"]
        self.landmark_gnss = rospy.get_param('/obstacle/landmark_gnss')
        self.tf_br = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.obstacle_msg = ObstacleArrayMsg() 
        self.marker_data = MarkerArray()
        #self.landmark_msg = AprilTagDetectionArray()
        self.odom = Odometry()
        self.start = PoseWithCovarianceStamped()
        self.position = PoseWithCovarianceStamped()
        self.now = rospy.get_rostime()
        self.prev = rospy.get_rostime()

    def make_msg(self, Depth1Image, Depth2Image, detection_data, camera1_param, camera2_param, odom, start):
        bboxes_from_camera1 = BoundingBoxes()
        bboxes_from_camera2 = BoundingBoxes()
        bboxes_from_camera1.header = Depth1Image.header
        bboxes_from_camera2.header = Depth2Image.header
        self.odom = odom
        self.start = start
        self.now = rospy.get_rostime()
        self.timebefore = detection_data.header.stamp
        # Add point obstacle
        self.obstacle_msg.header.stamp = detection_data.header.stamp
        self.obstacle_msg.header.frame_id = "odom" # CHANGE HERE: odom/map
        #opencvに変換
        bridge = CvBridge()
        try:
            Depth1image = bridge.imgmsg_to_cv2(Depth1Image, desired_encoding="passthrough")
            Depth2image = bridge.imgmsg_to_cv2(Depth2Image, desired_encoding="passthrough")
            for bbox in detection_data.bounding_boxes:
                if (bbox.xmax - bbox.xmin) > 50 and (bbox.ymax - bbox.ymin) > 50:
                    tmp_box = BoundingBox()
                    tmp_box.Class = bbox.Class
                    tmp_box.probability = bbox.probability
                    tmp_box.xmin = bbox.xmin
                    tmp_box.xmax = bbox.xmax
                    tmp_box.ymin = bbox.ymin
                    tmp_box.ymax = bbox.ymax
                    if (tmp_box.ymin + bbox.ymax)/2 < Depth1image.shape[0]:
                        if tmp_box.ymax > Depth1image.shape[0]:
                            tmp_box.ymax = Depth1image.shape[0]
                        bboxes_from_camera1.bounding_boxes.append(tmp_box)
                    elif  (bbox.ymin + bbox.ymax)/2 > Depth1image.shape[0]:
                        tmp_box.ymin = bbox.ymin - Depth1image.shape[0]
                        tmp_box.ymax = bbox.ymax - Depth1image.shape[0]
                        if bbox.ymin < 0:
                            tmp_box.ymin = 0
                        bboxes_from_camera2.bounding_boxes.append(tmp_box)
            camera1_obstacle_msg, camera2_obstacle_msg = ObstacleArrayMsg(), ObstacleArrayMsg()
            camera1_marker_data, camera2_marker_data = MarkerArray(), MarkerArray()
            camera1_obstacle_msg, camera1_marker_data = self.bbox_to_position_in_odom(bboxes_from_camera1, Depth1image, camera1_param)
            obstacle_msg, marker_data = self.bbox_to_position_in_odom(bboxes_from_camera2, Depth2image, camera2_param, len(camera1_obstacle_msg.obstacles), camera1_obstacle_msg, camera1_marker_data)
            self.obstacle_msg.obstacles, self.marker_data.markers = self.update_obstacles(self.obstacle_msg, obstacle_msg, self.marker_data, marker_data)
        except CvBridgeError as e:
            pass

    def send_msg(self):
        self.obstacle_msg.obstacles = [i for i in self.obstacle_msg.obstacles if not len(i.polygon.points) == 0]
        self.publisher.publish(self.obstacle_msg)
        self.marker_publisher.publish(self.marker_data)
    
    def bbox_to_position_in_odom(self, bboxes, DepthImage, camera_param, i=0, obstacle_msg=ObstacleArrayMsg(), marker_data=MarkerArray()):
        j = 0
        self.position = PoseWithCovarianceStamped()
        self.marker_data = MarkerArray()
        if i == 0:
            del obstacle_msg.obstacles[:]
            del marker_data.markers[:]
        for bbox in bboxes.bounding_boxes:
            if bbox.Class in self.obstacle_list:
                try:
                    tan_angle_x = camera_param[0][0]*(bbox.xmin+bbox.xmax)/2+camera_param[0][1]*(bbox.ymin+bbox.ymax)/2+camera_param[0][2]*1 
                    angle_x = math.atan(tan_angle_x)
                    if 0 < abs(math.degrees(angle_x)) < 30:
                        detected_area = DepthImage[bbox.ymin:bbox.ymax, bbox.xmin:bbox.xmax]
                        detected_area = np.where(detected_area > 8.0, detected_area, np.nan)
                        distance_x = np.nanmedian(detected_area)/1000
                        distance_x = distance_x + 0.1
                        distance_y = - distance_x * tan_angle_x
                        if 0.5 < distance_x < 3:
                            now = rospy.Time.now()
                            obstacle_msg.obstacles.append(ObstacleMsg())
                            marker_data.markers.append(Marker())
                            self.tf_br.sendTransform((-distance_y, 0, distance_x), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), bbox.Class + str(i), bboxes.header.frame_id)
                            self.tf_listener.waitForTransform("/odom", "/" + bbox.Class + str(i), rospy.Time(0), rospy.Duration(0.1))
                            obstable_position = self.tf_listener.lookupTransform("/odom", "/" + bbox.Class + str(i), now)
                            obstacle_msg.obstacles[i].header.stamp, obstacle_msg.obstacles[i].header.frame_id = bboxes.header.stamp, "odom"
                            obstacle_msg.obstacles[i].id = i
                            obstacle_msg.obstacles[i].polygon.points = [Point32()]
                            obstacle_msg.obstacles[i].polygon.points[0].x = obstable_position[0][0]
                            obstacle_msg.obstacles[i].polygon.points[0].y = obstable_position[0][1]
                            obstacle_msg.obstacles[i].polygon.points[0].z = obstable_position[0][2]
                            marker_data.markers[i].header.stamp, marker_data.markers[i].header.frame_id = bboxes.header.stamp, "odom"
                            marker_data.markers[i].ns, marker_data.markers[i].id = "obstacle", i
                            marker_data.markers[i].action = Marker.ADD
                            marker_data.markers[i].pose.position.x, marker_data.markers[i].pose.position.y, marker_data.markers[i].pose.position.z = obstable_position[0][0], obstable_position[0][1], obstable_position[0][2]
                            marker_data.markers[i].pose.orientation.x, marker_data.markers[i].pose.orientation.y, marker_data.markers[i].pose.orientation.z, marker_data.markers[i].pose.orientation.w= tf.transformations.quaternion_from_euler(0, 0, 0) 
                            marker_data.markers[i].color.r, marker_data.markers[i].color.g, marker_data.markers[i].color.b, marker_data.markers[i].color.a = 0, 1, 0, 1
                            marker_data.markers[i].scale.x, marker_data.markers[i].scale.y, marker_data.markers[i].scale.z = 0.2, 0.2, 1
                            marker_data.markers[i].type = 3
                            i = i + 1
                except Exception as e:
                    print(e)
            if bbox.Class in self.landmark_list:
                #self.landmark_msg = AprilTagDetectionArray()
                #self.landmark_msg.header = bboxes.header
                #self.landmark_msg.header.stamp = bboxes.header.stamp
                #self.landmark_msg.header.frame_id = bboxes.header.frame_id
                try:
                    tan_angle_x = camera_param[0][0]*((bbox.xmin+bbox.xmax)/2) + camera_param[0][2]*1 
                    angle_x = math.atan(tan_angle_x)
                    if abs(math.degrees(angle_x)) < 30:
                        detected_area = DepthImage[bbox.ymin:bbox.ymax, bbox.xmin:bbox.xmax]
                        #detected_area = np.where(detected_area > 8.0, detected_area, np.nan)
                        distance_x = distance_x + 0.1
                        distance_x = np.nanmedian(detected_area)/1000
                        distance_y = - distance_x * tan_angle_x
                        distance_e = (distance_x * distance_x + distance_y * distance_y)**0.5
                        if 1.0 < distance_x <3.5:
                            now = rospy.Time.now()
                            #distance_x = distance_x + 0.1 #self.landmark_msg.detections.append(AprilTagDetection())
                            landmark_name = "/" + bbox.Class + bboxes.header.frame_id + str(now)
                            self.tf_br.sendTransform((-distance_y, 0, distance_x), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(),landmark_name ,bboxes.header.frame_id)
                            self.position = PoseWithCovarianceStamped()
                            self.position.header = self.odom.header
                            self.position.header.frame_id = "base_link"
                            base_link_name = "/base_link" + str(now)
                            base_link_position = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
                            self.tf_br.sendTransform(base_link_position[0], tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0), now, base_link_name ,"map")
                            self.tf_br.sendTransform((-distance_y, 0, distance_x), tf.transformations.quaternion_from_euler(0, 0, 0), now, bbox.Class + str(i), bboxes.header.frame_id)
                            self.tf_listener.waitForTransform(base_link_name, "/" + bbox.Class + str(i), now, rospy.Duration(0.1))
                            gnss_position = self.tf_listener.lookupTransform(base_link_name, "/" + bbox.Class + str(i), rospy.Time(0))
                            self.position.pose.pose.orientation.x, self.position.pose.pose.orientation.y, self.position.pose.pose.orientation.z, self.position.pose.pose.orientation.w = base_link_position[1][0], base_link_position[1][1], base_link_position[1][2], base_link_position[1][3]
                            orientation = [self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, - self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w]
                            e = tf.transformations.euler_from_quaternion(orientation)
                            dist = np.array([])
                            position = np.array([])
                            for landmark in self.landmark_gnss:
                                dist_x = landmark[0] - self.start.pose.pose.position.x - gnss_position[0][0] - base_link_position[0][0]
                                dist_y = landmark[1] - self.start.pose.pose.position.y - gnss_position[0][1] - base_link_position[0][1]
                                dist = np.append(dist, dist_x * dist_x + dist_y * dist_y)
                                position = np.append(position,  np.array([landmark[0] - self.start.pose.pose.position.x, landmark[1] - self.start.pose.pose.position.y]), axis=0)
                            min_index = dist.argmin()
                            gnss_x = position[2 * min_index]#self.landmark_gnss[0][0] - self.start.pose.pose.position.x
                            gnss_y = position[2 * min_index + 1]#self.landmark_gnss[0][1] - self.start.pose.pose.position.y
                            print(gnss_x)
                            print(gnss_y)
                            self.position.pose.pose.position.x = gnss_x - gnss_position[0][0]
                            self.position.pose.pose.position.y = gnss_y - gnss_position[0][1]
                            self.position.pose.pose.position.z = 0
                            self.position.pose.pose.orientation.x, self.position.pose.pose.orientation.y, self.position.pose.pose.orientation.z, self.position.pose.pose.orientation.w = base_link_position[1][0], base_link_position[1][1], base_link_position[1][2], base_link_position[1][3]
                            self.position.pose.covariance = [0.01 * distance_x, 0, 0, 0, 0, 0, 0, 0.02 * distance_x, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.01, 0, 0,0, 0, 0, 0, 0.1]
                            self.marker_data.markers.append(Marker())
                            self.marker_data.markers[0].header.stamp, self.marker_data.markers[0].header.frame_id = bboxes.header.stamp, "map"     
                            self.marker_data.markers[0].ns, self.marker_data.markers[0].id = bbox.Class, 0
                            self.marker_data.markers[0].action = Marker.ADD
                            self.marker_data.markers[0].pose.position.x, self.marker_data.markers[0].pose.position.y, self.marker_data.markers[0].pose.position.z = gnss_x, gnss_y, 0.5
                            self.marker_data.markers[0].pose.orientation.x, self.marker_data.markers[0].pose.orientation.y, self.marker_data.markers[0].pose.orientation.z, self.marker_data.markers[0].pose.orientation.w= tf.transformations.quaternion_from_euler(0, 0, 0) 
                            self.marker_data.markers[0].color.r, self.marker_data.markers[0].color.g, self.marker_data.markers[0].color.b, self.marker_data.markers[0].color.a = 1, 0, 0, 1
                            self.marker_data.markers[0].scale.x, self.marker_data.markers[0].scale.y, self.marker_data.markers[0].scale.z = 0.2, 0.2, 1
                            self.marker_data.markers[0].type = 3
                            self.marker_data.markers.append(Marker())
                            self.marker_data.markers[1].header.stamp, self.marker_data.markers[1].header.frame_id = bboxes.header.stamp, "map"     
                            self.marker_data.markers[1].ns, self.marker_data.markers[1].id = bbox.Class, 1
                            self.marker_data.markers[1].action = Marker.ADD
                            self.marker_data.markers[1].pose.position.x, self.marker_data.markers[1].pose.position.y, self.marker_data.markers[1].pose.position.z = self.position.pose.pose.position.x, self.position.pose.pose.position.y, 0.0
                            self.marker_data.markers[1].pose.orientation.x, self.marker_data.markers[1].pose.orientation.y, self.marker_data.markers[1].pose.orientation.z, self.marker_data.markers[1].pose.orientation.w= tf.transformations.quaternion_from_euler(0, 0, 0) 
                            self.marker_data.markers[1].color.r, self.marker_data.markers[1].color.g, self.marker_data.markers[1].color.b, self.marker_data.markers[1].color.a = 1, 0, 0, 1
                            self.marker_data.markers[1].scale.x, self.marker_data.markers[1].scale.y, self.marker_data.markers[1].scale.z = 0.5, 0.5, 0.5
                            self.marker_data.markers[1].type = 1
                            if (rospy.Time.now()-now) < rospy.Duration(0.1):
                                #self.gnss_publisher.publish(self.position)
                                self.marker_publisher.publish(self.marker_data)
                            j = j + 1
                except Exception as e:
                    pass
        return obstacle_msg, marker_data
    """
    def update_obstacles(self, prev_obstacle_msg, detected_obstacle_msg, prev_marker_msg, marker_msg):
        self.tf_listener.waitForTransform("/odom", "/base_link", rospy.Time(0), rospy.Duration(0.1))
        current_position = self.tf_listener.lookupTransform("/odom", "/base_link",  rospy.Time(0))
        print(len(prev_obstacle_msg.obstacles))    
        for detected_obstacle, marker in zip(detected_obstacle_msg.obstacles, marker_msg.markers): 
            updated = False
            for prev_obstacle, prev_marker in zip(prev_obstacle_msg.obstacles, prev_marker_msg.markers):  
                if abs(current_position[0][0] - prev_obstacle.polygon.points[0].x) > 4 or abs(current_position[0][1] - prev_obstacle.polygon.points[0].y) > 4:
                    prev_obstacle_msg.obstacles.pop(prev_obstacle)
                    prev_marker_msg.markers.pop(prev_marker)  
                    break
                if ((detected_obstacle.polygon.points[0].x - prev_obstacle.polygon.points[0].x) * (detected_obstacle.polygon.points[0].x - prev_obstacle.polygon.points[0].x) + (detected_obstacle.polygon.points[0].y - prev_obstacle.polygon.points[0].y) * (detected_obstacle.polygon.points[0].y - prev_obstacle.polygon.points[0].y)) < 1:
                    prev_obstacle.polygon.points[0].x = detected_obstacle.polygon.points[0].x 
                    prev_obstacle.polygon.points[0].y = detected_obstacle.polygon.points[0].y
                    print(prev_marker.pose.position.x)
                    prev_marker.pose.position.x = marker.pose.position.x 
                    print(prev_marker.pose.position.x)
                    prev_marker.pose.position.y = marker.pose.position.y
                    updated = True                  
                break
                if not updated:
                    prev_obstacle_msg.obstacles.append(detected_obstacle)                    
                    prev_marker_msg.markers.append(marker)                    
        return prev_obstacle_msg.obstacles, prev_marker_msg.markers
        """
    def update_obstacles(self, prev_obstacle_msg, detected_obstacle_msg, prev_marker_msg, marker_msg):
        self.tf_listener.waitForTransform("/odom", "/base_link", rospy.Time(0), rospy.Duration(0.1))
        current_position = self.tf_listener.lookupTransform("/odom", "/base_link",  rospy.Time(0))
        updated_obstacle_msg = ObstacleArrayMsg() 
        updated_marker_data = MarkerArray()
        prev_obstacle_in_area = ObstacleArrayMsg()
        prev_marker_in_area = MarkerArray() 
        try:
            for i, prev_obstacle in enumerate(self.obstacle_msg.obstacles):
                if abs(current_position[0][0] - prev_obstacle.polygon.points[0].x) < 3 and abs(current_position[0][1] - prev_obstacle.polygon.points[0].y) < 3:
                    prev_obstacle_in_area.obstacles.append(prev_obstacle)
            self.obstacle_msg.obstacles = prev_obstacle_in_area.obstacles
        except Exception as e:
            pass
        for detected_obstacle in detected_obstacle_msg.obstacles:
            if len(prev_obstacle_in_area.obstacles) > 0:
                num = len(self.obstacle_msg.obstacles)
                Updated = False
                for i in range(num):
                    try:
                        if ((detected_obstacle.polygon.points[0].x - self.obstacle_msg.obstacles[i].polygon.points[0].x) * (detected_obstacle.polygon.points[0].x - self.obstacle_msg.obstacles[i].polygon.points[0].x) + (detected_obstacle.polygon.points[0].y - self.obstacle_msg.obstacles[i].polygon.points[0].y) * (detected_obstacle.polygon.points[0].y - self.obstacle_msg.obstacles[i].polygon.points[0].y)) < 2.25:
                            self.obstacle_msg.obstacles[i].header.stamp.secs = 0
                            self.obstacle_msg.obstacles[i].header.stamp.nsecs = 0
                            self.obstacle_msg.obstacles[i].polygon.points[0].x = detected_obstacle.polygon.points[0].x 
                            self.obstacle_msg.obstacles[i].polygon.points[0].y = detected_obstacle.polygon.points[0].y
                            #prev_marker.pose.position.x = marker.pose.position.x 
                            #prev_marker.pose.position.y = marker.pose.position.y
                            #if not len(detected_obstacle.polygon.points) == 0:
                            #updated_obstacle_msg.obstacles.append(detected_obstacle)  
                            #print("1")                  
                            #updated_marker_data.markers.append(prev_marker)    
                            Updated = True
                    except Exception as e:
                        print(e)
                if not Updated:     
                    if not len(detected_obstacle.polygon.points) == 0:
                        self.obstacle_msg.obstacles.append(detected_obstacle)   
            else:     
                self.obstacle_msg.obstacles.append(detected_obstacle)   
        self.obstacle_msg.obstacles = list(set(self.obstacle_msg.obstacles))
        return self.obstacle_msg.obstacles, marker_msg.markers

class Subscribe_publishers():
    def __init__(self, pub):
        self.depth1_subscriber = message_filters.Subscriber('/camera1/aligned_depth_to_color/image_raw', Image)
        self.depth2_subscriber = message_filters.Subscriber('/camera2/aligned_depth_to_color/image_raw', Image)
        self.detection_subscriber =message_filters.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes)
        self.odom_subscriber = message_filters.Subscriber('/odometry/filtered', Odometry)
        self.camera1_param_subscriber = rospy.Subscriber("/camera1/color/camera_info", CameraInfo, self.camera1_parameter_callback)
        self.camera2_param_subscriber = rospy.Subscriber("/camera2/color/camera_info", CameraInfo, self.camera2_parameter_callback)
        self.start_subscriber = rospy.Subscriber('/set_start', Odometry, self.gnss_start_callback)
        #self.start_subscriber = rospy.Subscriber('/gnss_odom', Odometry, self.gnss_start_callback)

        # messageの型を作成
        self.pub = pub
        self.depth1_image = Image()
        self.depth2_image = Image()
        self.detection_data = BoundingBoxes()
        self.camera1_parameter = CameraInfo()
        self.camera2_parameter = CameraInfo()
        self.odom = Odometry()
        self.start_position = PoseWithCovarianceStamped()
        ts = message_filters.ApproximateTimeSynchronizer([self.depth1_subscriber, self.depth2_subscriber, self.detection_subscriber, self.odom_subscriber], 10, 0.07)
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

    def gnss_start_callback(self, Odom):
        start = Odom
        start.header.frame_id = "base_link"
        self.start_position.pose.pose.position.x = start.pose.pose.position.x
        self.start_position.pose.pose.position.y = start.pose.pose.position.y
        self.start_position.pose.pose.orientation = start.pose.pose.orientation
        print ("start_x:" + str(start.pose.pose.position.x))
        print ("start_y:" + str(start.pose.pose.position.y))
        self.start_subscriber.unregister() 
    """
    def gnss_start_callback(self, Odom):
        #if  rospy.get_time() < 1566368048.486150979:
        if  rospy.get_time() < 1566367515.781358003:
            start = Odom
            start.header.frame_id = "base_link"
            self.start_position.pose.pose.position.x = start.pose.pose.position.x
            self.start_position.pose.pose.position.y = start.pose.pose.position.y
            self.start_position.pose.pose.orientation = start.pose.pose.orientation
            print ("start_x:" + str(start.pose.pose.position.x))
            print ("start_y:" + str(start.pose.pose.position.y))
            #self.start_subscriber.unregister() 
    """
    def bounding_boxes_callback(self, depth1_image, depth2_image, detection_data, odom):
        self.depth1_image, self.depth2_image, self.detection_data, self.odom = depth1_image, depth2_image, detection_data, odom

    def pub_msg(self):
        self.pub.make_msg(self.depth1_image, self.depth2_image, self.detection_data, self.camera1_parameter, self.camera2_parameter, self.odom, self.start_position)
        self.pub.send_msg()

def main():
    # nodeの立ち上げ
    rospy.init_node('obstacle')
    rospy.sleep(5)
    # クラスの作成
    pub = Publishsers()
    sub = Subscribe_publishers(pub)
    while not rospy.is_shutdown():
        sub.pub_msg()

if __name__ == '__main__':
    main()
