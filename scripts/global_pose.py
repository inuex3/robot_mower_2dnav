#!/usr/bin/env python
import rospy
import tf
import numpy as np
from tf.transformations import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rtabmap_ros.srv import ResetPose
import csv
reset_pose = rospy.ServiceProxy('/rtabmap/reset_odom_to_pose', ResetPose)

n = 0
start = Odometry()
pose_pub = rospy.Publisher('/rtabmap/global_pose', PoseWithCovarianceStamped, queue_size=1)
pub = rospy.Publisher('/gnss_odom_from_start', Odometry, queue_size=1)
prev = Odometry()
f = open('test.csv','a')


def callback_odom(Odom):
    global n
    global start
    global prev
    gnss_odom = Odometry()
    posi = PoseWithCovarianceStamped()
    from_num = 23871
    #from_time = 1566368048.486150979
    #end_time = 1566368551.485430955
    from_time = 1566367515.781358003
    end_time =1566367846.78736066
    listener = tf.TransformListener()
    #if Odom.header.seq == from_num:#23868:#23871:#25975:
    if rospy.get_time() < from_time:#23868:#23871:#25975:
        start = Odom
        start.header.frame_id = "map"
        e = tf.transformations.euler_from_quaternion((start.pose.pose.orientation.x, start.pose.pose.orientation.y, start.pose.pose.orientation.z, start.pose.pose.orientation.w))
        reset_pose(0.0, 0.0, 0.0, 0, 0,e[2] + 0.045)#ouhuku
        posi.header.stamp = rospy.Time.now()
        #posi.pose.pose.position.x = 0.0
        #posi.pose.pose.position.y = 0.0
        posi.pose.pose.position.x = (Odom.pose.pose.position.x - start.pose.pose.position.x)
        posi.pose.pose.position.y = (Odom.pose.pose.position.y - start.pose.pose.position.y)
        print(posi.pose.pose.position.x)
        print(posi.pose.pose.position.y)
        posi.header.frame_id = "base_link"
        posi.pose.pose.orientation = start.pose.pose.orientation
        posi.pose.covariance = [0.0001, 0, 0, 0, 0, 0, 0, 0.0001, 0, 0, 0, 0, 0, 0, 9999, 0, 0, 0, 0, 0, 0, 9999, 0, 0, 0, 0, 0, 0, 9999, 0, 0, 0, 0, 0, 0, 9999]
        pose_pub.publish(posi)
        n = 1
    if from_time < rospy.get_time() < end_time:#23868:#ouhuku2 23900:
        e = tf.transformations.euler_from_quaternion((start.pose.pose.orientation.x, start.pose.pose.orientation.y, start.pose.pose.orientation.z, start.pose.pose.orientation.w))
        yaw = e[2]
        gnss_odom.header.frame_id = "map"
        gnss_odom.pose.pose.position.x = (Odom.pose.pose.position.x - start.pose.pose.position.x)
        gnss_odom.pose.pose.position.y = (Odom.pose.pose.position.y - start.pose.pose.position.y)
        #orientation = quaternion_multiply((Odom.pose.pose.orientation.x,Odom.pose.pose.orientation.y,Odom.pose.pose.orientation.z,Odom.pose.pose.orientation.w), (start.pose.pose.orientation.x,start.pose.pose.orientation.y,start.pose.pose.orientation.z,start.pose.pose.orientation.w)) 
        #gnss_odom.pose.pose.orientation.x, gnss_odom.pose.pose.orientation.y, gnss_odom.pose.pose.orientation.z, gnss_odom.pose.pose.orientation.w = orientation[0], orientation[1], orientation[2], orientation[3]
        gnss_odom.pose.pose.orientation = Odom.pose.pose.orientation
        pub.publish(gnss_odom)
        prev = gnss_odom

def listen():
    rospy.Subscriber('/gnss_odom', Odometry, callback_odom)
    rospy.spin()

def main():
    rospy.init_node('set_pose')
    listen()

if __name__ == '__main__':
    main()


