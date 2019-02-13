#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn
import tf
import math
from nav_msgs.msg import Odometry

class Publishsers():
    def __init__(self):
        # Publisherを作成
        self.publisher = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size = 1)
        # messageの型を作成
        self.RC_msg = OverrideRCIn()
        self.cmd_vel = Twist()
        self.current_time = rospy.get_time()+0.1
        self.prev_time = rospy.get_time()
        self.prev_odom = Odometry()
        self.P_gain_x = 300
        self.P_gain_theta = 100

    def make_msg(self, cmd_vel, odom):
        # 処理を書く
        self.current_time = rospy.get_time()
        if (cmd_vel.angular.z > 0):
            theta = 100
        elif (cmd_vel.angular.z < 0):
            theta = -100
        elif (cmd_vel.angular.z == 0):
            theta = 0
        if (cmd_vel.linear.x > 0):
            x = 50
        elif (cmd_vel.linear.x < 0):
            x = -80
        elif (cmd_vel.linear.x == 0):
            x = 0                
        self.RC_msg.channels = [1500 - theta - (cmd_vel.angular.z - odom.twist.twist.angular.z)*self.P_gain_theta, 1500 - x - (cmd_vel.linear.x - odom.twist.twist.linear.x)*self.P_gain_theta, 0, 0, 0, 0, 0, 0] #1:steering,2:thrust    
        
    def send_msg(self, current_time, cb_time):
        if (current_time - cb_time) > 2.0:
            self.RC_msg.channels = [0, 0, 0, 0, 0, 0, 0, 0] #1:steering,2:thrust   
        self.publisher.publish(self.RC_msg)

class Subscribe_publishers():
    def __init__(self, pub):
        # Subscriberを作成
        self.subscriber = rospy.Subscriber('/cmd_vel', Twist, self.callback, queue_size = 1)
        #self.odom_subscriber = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # messageの型を作成
        # publish
        self.pub = pub
        self.odom = Odometry()
        self.cb_time = rospy.get_time()  

    def callback(self, cmd_vel):
        self.cb_time = rospy.get_time()
        self.pub.make_msg(cmd_vel, self.odom)
        
    def odom_callback(self, msg):
        #callback時の処理
        self.odom = msg

    def send(self, current_time):
        current_time = rospy.get_time()        
        self.pub.send_msg(current_time, self.cb_time)

def main():
    # nodeの立ち上げ
    rospy.init_node('cmd_to_RC')

    # クラスの作成
    pub = Publishsers()
    sub = Subscribe_publishers(pub)
    
    while not rospy.is_shutdown():
        current_time = rospy.get_time()
        sub.send(current_time)
        rospy.sleep(0.1)

if __name__ == '__main__':
    main()
