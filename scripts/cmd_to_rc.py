#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn
import tf
from nav_msgs.msg import Odometry
import math

class Publishsers():
    def __init__(self):
        # Publisherを作成
        self.publisher = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size = 1)
        # messageの型を作成
        self.RC_msg = OverrideRCIn()
        self.prev_RC_msg = OverrideRCIn()
        self.cmd_vel = Twist()
        self.prev_odom = Odometry()
        self.gain_x = 100
        self.gain_theta = 100      

    def make_msg(self, cmd_vel, odom):
        # 処理を書く
        if (cmd_vel.angular.z > 0):
            theta = 1450
        elif (cmd_vel.angular.z < 0):
            theta = 1550
        elif (cmd_vel.angular.z == 0.0):
            theta = 1500
        if (cmd_vel.linear.x > 0):
            x = 1400
        elif (cmd_vel.linear.x < 0):
            x = 1600
        elif (cmd_vel.linear.x == 0.0):
            x = 1500 
        RC_theta = cmd_vel.angular.z * self.gain_theta
        RC_x = cmd_vel.linear.x * self.gain_x
        self.RC_msg.channels = [theta - RC_theta, x - RC_x, 0, 0, 0, 0, 0, 0] #1:steering,2:thrust
        
    def send_msg(self, time_now, cb_time):
        if (time_now - cb_time) > 1.5:
            self.RC_msg.channels = [0, 0, 0, 0, 0, 0, 0, 0] #1:steering,2:thrust
            self.prev_RC_theta = 0   
            self.prev_RC_x = 0
        self.publisher.publish(self.RC_msg)        

class Subscribe_publishers():
    def __init__(self, pub):
        # Subscriberを作成
        self.subscriber = rospy.Subscriber('/cmd_vel', Twist, self.callback, queue_size = 1)
        self.odom_subscriber = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback, queue_size = 30)
        # messageの型を作成
        self.pub = pub
        self.odom = Odometry()
        self.cmd_vel = Twist()
        self.cb_time = rospy.get_time() 

    def callback(self, cmd_vel):
        self.cb_time = rospy.get_time() 
        self.cmd_vel = cmd_vel       
    
    def odom_callback(self, odom):
        #callback時の処理
        self.odom = odom

    def send_msg(self, current_time):
        self.pub.make_msg(self.cmd_vel, self.odom)
        self.pub.send_msg(current_time, self.cb_time)        

def main():
    # nodeの立ち上げ
    rospy.init_node('cmd_to_RC')

    # クラスの作成
    pub = Publishsers()
    sub = Subscribe_publishers(pub)

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        time_now = rospy.get_time()
        sub.send_msg(time_now)

if __name__ == '__main__':
    main()
