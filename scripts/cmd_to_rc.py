#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn
import tf
from nav_msgs.msg import Odometry

class Publishsers():
    def __init__(self):
        # Publisherを作成
        self.publisher = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size = 1)
        # messageの型を作成
        self.RC_msg = OverrideRCIn()
        self.cmd_vel = Twist()
        self.current_time = 0
        self.prev_time = 0
        self.prev_odom = Odometry()
        self.P_gain_x = 30
        self.P_gain_theta = 20
        self.prev_RC_theta = 0
        self.prev_RC_x = 0

    def make_msg(self, cmd_vel, odom):
        # 処理を書く
        self.current_time = rospy.get_time()
        self.prev_odom.pose.pose.orientation.w = - self.prev_odom.pose.pose.orientation.w
        orientation_diff = tf.transformations.quaternion_multiply((odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w),(self.prev_odom.pose.pose.orientation.x, self.prev_odom.pose.pose.orientation.y, self.prev_odom.pose.pose.orientation.z, self.prev_odom.pose.pose.orientation.w))       
        theta_diff = tf.transformations.euler_from_quaternion(orientation_diff)
        if (cmd_vel.angular.z > 0):
            theta = 1450
        elif (cmd_vel.angular.z < 0):
            theta = 1550
        elif (cmd_vel.angular.z == 0):
            theta = 0.0
        if (cmd_vel.linear.x > 0):
            x = 1450
        elif (cmd_vel.linear.x < 0):
            x = 1550
        elif (cmd_vel.linear.x == 0):
            x = 0.0    
        print("")    
        print((odom.header.stamp - self.prev_odom.header.stamp)/10.0**8)
        print(self.prev_odom.header.stamp)         
        print("")                     
        print("")                     
        RC_theta = self.prev_RC_theta# + (cmd_vel.angular.z - theta_diff[2]/(self.current_time - self.prev_time))*self.P_gain_theta
        #RC_theta = self.prev_RC_theta + (cmd_vel.angular.z - theta_diff[2]/(odom. - self.prev_odom))*self.P_gain_theta
        RC_x = self.prev_RC_x #+ (cmd_vel.angular.z - theta_diff[2]/(self.current_time - self.prev_time))*self.P_gain_theta
        self.RC_msg.channels = [theta - RC_theta, x - cmd_vel.linear.x*self.P_gain_theta, 0, 0, 0, 0, 0, 0] #1:steering,2:thrust
        #print("cmd"+str(cmd_vel.angular.z))
        #print("vel"+str(normalized_vel_theta[2]/(self.current_time - self.prev_time)))
        #print("diff"+str(cmd_vel.angular.z-normalized_vel_theta[2]/(self.current_time - self.prev_time)))
        self.prev_RC_theta = RC_theta
        self.prev_odom = odom
        self.prev_time = self.current_time       
        
    def send_msg(self, time_now, cb_time):
        if (time_now - cb_time) > 2.0:
            self.RC_msg.channels = [0, 0, 0, 0, 0, 0, 0, 0] #1:steering,2:thrust
            self.prev_RC_theta = 0   
            self.prev_RC_x = 0
        print("RC"+str(self.RC_msg.channels))            
        self.publisher.publish(self.RC_msg)        

class Subscribe_publishers():
    def __init__(self, pub):
        # Subscriberを作成
        self.subscriber = rospy.Subscriber('/cmd_vel', Twist, self.callback, queue_size = 1)
        #self.odom_subscriber = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback, queue_size = 1)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size = 1)
        # messageの型を作成
        self.pub = pub
        self.odom = Odometry()
        self.cmd_vel = Twist()
        self.cb_time = rospy.get_time()        

    def callback(self, msg):
        self.cb_time = rospy.get_time()        
        self.cmd_vel = msg        
    
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
    r = rospy.Rate(100) # 10hz

    while not rospy.is_shutdown():
        r.sleep()
        time_now = rospy.get_time()
        sub.send_msg(time_now)

if __name__ == '__main__':
    main()
