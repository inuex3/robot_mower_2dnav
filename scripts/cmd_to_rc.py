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
        self.current_time = rospy.get_time()+0.1
        self.prev_time = rospy.get_time()
        self.prev_odom = Odometry()
        self.P_gain_x = 300
        self.P_gain_theta = 100

    def make_msg(self, cmd_vel, odom):
        # 処理を書く
        self.current_time = rospy.get_time()
        if ((self.current_time - self.prev_time) == 0.0):
            pass
        else: 
            self.prev_odom.pose.pose.orientation.w = - self.prev_odom.pose.pose.orientation.w
            orientation_diff = tf.transformations.quaternion_multiply((odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w),(self.prev_odom.pose.pose.orientation.x, self.prev_odom.pose.pose.orientation.y, self.prev_odom.pose.pose.orientation.z, self.prev_odom.pose.pose.orientation.w))       
            normalized_vel_theta = tf.transformations.euler_from_quaternion(orientation_diff)
            if (cmd_vel.angular.z > 0):
                theta = 50
            elif (cmd_vel.angular.z < 0):
                theta = -50
            elif (cmd_vel.angular.z == 0):
                theta = 0
            if (cmd_vel.linear.x > 0):
                x = 50
            elif (cmd_vel.linear.x < 0):
                x = -80
            elif (cmd_vel.linear.x == 0):
                x = 0                
            self.RC_msg.channels = [1500 - theta - (cmd_vel.angular.z - normalized_vel_theta[2])/(self.current_time - self.prev_time)*self.P_gain_theta, 1500 - x - cmd_vel.linear.x*self.P_gain_theta, 0, 0, 0, 0, 0, 0] #1:steering,2:thrust
            #print("cmd"+str(cmd_vel.angular.z))
            #print("vel"+str(normalized_vel_theta[2]/(self.current_time - self.prev_time)))
            #print("diff"+str(cmd_vel.angular.z-normalized_vel_theta[2]/(self.current_time - self.prev_time)))
            print("RC"+str(self.RC_msg.channels))
        self.prev_odom = odom
        self.prev_time = self.current_time       
        self.publisher.publish(self.RC_msg)

class Subscribe_publishers():
    def __init__(self, pub):
        # Subscriberを作成
        self.subscriber = rospy.Subscriber('/cmd_vel', Twist, self.callback, queue_size = 1)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # messageの型を作成
        # publish
        self.pub = pub
        self.odom = Odometry()

    def callback(self, cmd_vel):
        #callback時の処理
        self.pub.make_msg(cmd_vel, self.odom)

    def odom_callback(self, msg):
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
