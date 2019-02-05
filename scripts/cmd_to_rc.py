#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn
import tf

import tf
from nav_msgs.msg import Odometry

class Publishsers():
    def __init__(self):
        # Publisherを作成
        self.publisher = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size = 1)
        #self.max_x = rospy.get_param("~coverage_spacing", 0.4)
        #self.min_x = rospy.get_param("~coverage_spacing", 0.4)
        self.max_vel_theta = 0.4#= rospy.get_param("~coverage_spacing", 0.4)
        # messageの型を作成
        self.RC_msg = OverrideRCIn()
        self.cmd_vel = Twist()
        self.current_time = rospy.Time.now()
        self.prev_time = rospy.Time.now()
        self.prev_odom = Odometry()
        self.P_gain_x = 200
        self.P_gain_theta = 200

    def make_msg(self, cmd_vel, odom):
        # 処理を書く
        self.current_time = rospy.Time.now()
        #normalized_vel_x = (odom.pose.position.x - self.prev_odom.pose.position.x)/ (self.current_time - self.prev_time)
        vel_rot = (odom.pose.orientation - self.prev_odom.pose.orientation)/ (self.current_time - self.prev_time)
        normalized_vel_theta = tf.transformations.euler_from_quaternion(vel_rot)
        print(normalized_vel_theta[2])
        self.RC_msg.channels = [normalized_vel_theta-cmd_vel.angular.z*angular_gain, 1400 - cmd_vel.linear.x*linear_gain, 0, 0, 0, 0, 0, 0] #1:steering,2:thrust
	print(self.RC_msg.channels)
        self.prev_odom = odom

    def send_msg(self):
        # messageを送信
        self.publisher.publish(self.RC_msg)

class Subscribe_publishers():
    def __init__(self, pub):
        # Subscriberを作成
        self.subscriber = rospy.Subscriber('/cmd_vel', Twist, self.callback, queue_size = 1)
        self.subscriber = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        # messageの型を作成
        # publish
	self.pub = pub
	self.odom = Odometry()

    def callback(self, cmd_vel):
        #callback時の処理
        self.pub.make_msg(cmd_vel, self.odom)
        self.pub.send_msg()

    def odom_callback(self, msg):
        print(msg)
        #callback時の処理
        self.odom = msg

def main():
    # nodeの立ち上げ
    rospy.init_node('cmd_to_RC')

    # クラスの作成
    pub = Publishsers()
    sub = Subscribe_publishers(pub)

    rospy.spin()

if __name__ == '__main__':
    main()
