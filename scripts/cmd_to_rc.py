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
        self.current_time = 0
        self.prev_time = 0
        self.prev_odom = Odometry()
        self.P_gain_x = 5
        self.P_gain_theta = 15
        self.prev_RC_theta = 0
        self.prev_RC_x = 0

    def make_msg(self, cmd_vel, odom):
        # 処理を書く
        self.current_time = rospy.get_time()
        self.prev_odom.pose.pose.orientation.w = - self.prev_odom.pose.pose.orientation.w
        orientation_diff = tf.transformations.quaternion_multiply((odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w),(self.prev_odom.pose.pose.orientation.x, self.prev_odom.pose.pose.orientation.y, self.prev_odom.pose.pose.orientation.z, self.prev_odom.pose.pose.orientation.w))       
        theta_diff = tf.transformations.euler_from_quaternion(orientation_diff)
        position_diff = ((odom.pose.pose.position.x - self.prev_odom.pose.pose.position.x)*(odom.pose.pose.position.x - self.prev_odom.pose.pose.position.x) + (odom.pose.pose.position.y - self.prev_odom.pose.pose.position.y) * (odom.pose.pose.position.y - self.prev_odom.pose.pose.position.y)) ** 0.5   
        if (cmd_vel.angular.z > 0):
            theta = 1450
        elif (cmd_vel.angular.z < 0):
            theta = 1550
        elif (cmd_vel.angular.z == 0.0):
            theta = 1500
        if (cmd_vel.linear.x > 0):
            x = 1400
        elif (cmd_vel.linear.x < 0):
            x = 1650
        elif (cmd_vel.linear.x == 0.0):
            x = 1500 
        if self.prev_RC_msg.channels[1] > 1500:
            position_diff = - position_diff
        RC_theta = self.prev_RC_theta + (cmd_vel.angular.z - theta_diff[2]/(self.current_time - self.prev_time))*self.P_gain_theta
        RC_x = self.prev_RC_x + (cmd_vel.linear.x - position_diff / (self.current_time - self.prev_time))*self.P_gain_x
        #print("x"+str(position_diff / (self.current_time - self.prev_time)))
        #print("z"+str(cmd_vel.angular.z - theta_diff[2]/(self.current_time - self.prev_time)))
        self.RC_msg.channels = [theta - RC_theta, x - RC_x, 0, 0, 0, 0, 0, 0] #1:steering,2:thrust
        self.prev_RC_theta = RC_theta
        self.prev_RC_x = RC_x
        self.prev_odom = odom
        self.prev_RC_msg = self.RC_msg
        self.prev_time = self.current_time
        
    def send_msg(self, time_now, cb_time):
        if (self.RC_msg.channels[0] < 1000 or self.RC_msg.channels[1] < 1000):
            self.RC_msg.channels = [1500, 1500, 0, 0, 0, 0, 0, 0] #1:steering,2:thrust
        if (time_now - cb_time) > 2.0:
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
