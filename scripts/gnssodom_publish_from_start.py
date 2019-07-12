#!/usr/bin/env python
import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry

n = 0
start = Odometry()
pub = rospy.Publisher('/gnss_odom_from_start', Odometry, queue_size=1)

def callback_odom(Odom):
    global n
    global start
    gnss_odom = Odometry()
    if n == 0:
        start = Odom
        start.header.frame_id = "map"
    else:
        e = tf.transformations.euler_from_quaternion((start.pose.pose.orientation.x, start.pose.pose.orientation.y, start.pose.pose.orientation.z, start.pose.pose.orientation.w))
        #e = tf.transformations.euler_from_quaternion((start.pose.pose.orientation.x, start.pose.pose.orientation.y, start.pose.pose.orientation.z, start.pose.pose.orientation.w))
        yaw = e[2]
        C = np.cos(yaw)
        S = np.sin(yaw)
        gnss_odom.header.frame_id = "map"
        gnss_odom.pose.pose.position.x = (Odom.pose.pose.position.x - start.pose.pose.position.x)
        gnss_odom.pose.pose.position.y = (Odom.pose.pose.position.y - start.pose.pose.position.y)
        pub.publish(gnss_odom)
    n = n + 1
    
def listen():
    rospy.Subscriber('/gnss_odom', Odometry, callback_odom)
    rospy.spin()

def main():
    rospy.init_node('set_start')
    listen()

if __name__ == '__main__':
    main()


