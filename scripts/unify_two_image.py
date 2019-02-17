#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import message_filters
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
import cv2


class Publishsers():
    def __init__(self):
        # Publisherを作成
        self.publisher = rospy.Publisher('/camera/color/image_raw', Image, queue_size = 5)
        self.unified_image = Image()        

    def make_msg(self, color1_image, color2_image):
	#opencvに変換
	bridge = CvBridge()
   	try:
   	    image1 = bridge.imgmsg_to_cv2(color1_image, desired_encoding="passthrough")
   	    image2 = bridge.imgmsg_to_cv2(color2_image, desired_encoding="passthrough")
            unified = cv2.vconcat([image1, image2])
        except CvBridgeError as e:
            print(e)
	self.unified_image = bridge.cv2_to_imgmsg(unified, encoding='passthrough')
	#Time stampとframe id
	self.unified_image.header.stamp = rospy.Time.now()
	self.unified_image.encoding = "rgb8"
	self.unified_image.header.frame_id = "camera_color_optical_frame"

    def send_msg(self):
        # messageを送信
        self.publisher.publish(self.unified_image)

class Subscribe_publishers():
    def __init__(self, pub):
        # Subscriberを作成
        self.color1_subscriber = message_filters.Subscriber("/camera1/color/image_raw", Image)
        self.color2_subscriber = message_filters.Subscriber("/camera2/color/image_raw", Image)
        ts = message_filters.ApproximateTimeSynchronizer([self.color1_subscriber, self.color2_subscriber], 10, 0.05)
        ts.registerCallback(self.callback)
        # messageの型を作成
	self.pub = pub

    def callback(self, color1_image, color2_image):
        self.pub.make_msg(color1_image, color2_image)
        self.pub.send_msg()

def main():
    # nodeの立ち上げ
    rospy.init_node('crop')
    # クラスの作成
    pub = Publishsers()
    sub = Subscribe_publishers(pub)

    rospy.spin()

if __name__ == '__main__':
    main()
