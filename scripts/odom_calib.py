#!/usr/bin/env python
import rospy
import tf
import math
from std_msgs.msg import String
from nav_msgs.msg import Odometry

pub = rospy.Publisher('/vo_calib', Odometry, queue_size=10)

def callback(odom):
    new_odom = Odometry()
    new_odom = odom
    new_odom.header.stamp = rospy.Time.now()
    new_odom.pose.pose.position.x = odom.pose.pose.position.x * 2
    new_odom.pose.pose.position.y = odom.pose.pose.position.y * 2
    new_odom.twist.twist.linear.x = odom.twist.twist.linear.x * 2
    new_odom.twist.twist.linear.x = odom.twist.twist.linear.x * 2
    new_odom.pose.pose.orientation = odom.pose.pose.orientation
    new_odom.pose.covariance = list(odom.pose.covariance)
    new_odom.twist.covariance = list(odom.twist.covariance)
    pub.publish(new_odom)
    
def listen():
    rospy.Subscriber('/vo', Odometry, callback)
    rospy.spin()

def main():
    rospy.init_node('subscriber')
    listen()

if __name__ == '__main__':
    main()



