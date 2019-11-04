#!/usr/bin/env python
import rospy
import tf
import numpy as np
from rtabmap_ros.srv import ResetPose
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from tf.transformations import *
from sensor_msgs.msg import NavSatFix
import message_filters


n = 0
start = Odometry()
reset_pose = rospy.ServiceProxy('/rtabmap/reset_odom_to_pose', ResetPose)
pub = rospy.Publisher('/gnss_odom_from_start', Odometry, queue_size=1)
pub_gps = rospy.Publisher('/rtabmap/gps/fix', NavSatFix, queue_size=1)

def callback(Odom, gps):
    global n
    global start
    gnss_odom = Odometry()
    e = tf.transformations.euler_from_quaternion((Odom.pose.pose.orientation.x, Odom.pose.pose.orientation.y, Odom.pose.pose.orientation.z, Odom.pose.pose.orientation.w))
    if Odom.header.seq == 21868:
        start = Odom
        start.header.frame_id = "map"
        start_pose = ResetPose()
        e = tf.transformations.euler_from_quaternion((start.pose.pose.orientation.x, start.pose.pose.orientation.y, start.pose.pose.orientation.z, start.pose.pose.orientation.w))
        start_pose.x, start_pose.y, start_pose.z, start_pose.roll, start_pose.pitch, start_pose.yaw = 0.0, 0.0, 0.0, e[0],e[1],e[2]  
        #reset_pose(0.0, 0.0, 0.0, 0, 0,e[2])
        #rospy.sleep(0.2)
        #reset_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        #pub_gps.publish(gps)
    elif Odom.header.seq > 21868:
        e = tf.transformations.euler_from_quaternion((start.pose.pose.orientation.x, start.pose.pose.orientation.y, start.pose.pose.orientation.z, start.pose.pose.orientation.w))
        yaw = e[2]
        gnss_odom.header.frame_id = "map"
        gnss_odom.pose.pose.position.x = (Odom.pose.pose.position.x - start.pose.pose.position.x)
        gnss_odom.pose.pose.position.y = (Odom.pose.pose.position.y - start.pose.pose.position.y)
        orientation = quaternion_multiply((Odom.pose.pose.orientation.x,Odom.pose.pose.orientation.y,Odom.pose.pose.orientation.z,Odom.pose.pose.orientation.w), (start.pose.pose.orientation.x,start.pose.pose.orientation.y,start.pose.pose.orientation.z,start.pose.pose.orientation.w)) 
        gnss_odom.pose.pose.orientation.x, gnss_odom.pose.pose.orientation.y, gnss_odom.pose.pose.orientation.z, gnss_odom.pose.pose.orientation.w = orientation[0], orientation[1], orientation[2], orientation[3]
        pub.publish(gnss_odom)
    n = n + 1
    
def listen():
    odom_sub = message_filters.Subscriber('/gnss_odom', Odometry)
    gps_sub = message_filters.Subscriber('/RTK_NavSatFix', NavSatFix)
    ts = message_filters.ApproximateTimeSynchronizer([odom_sub, gps_sub], 10, 0.2)
    ts.registerCallback(callback)
    rospy.spin()

def main():
    rospy.wait_for_service('/rtabmap/reset_odom_to_pose')
    rospy.init_node('start')
    listen()

if __name__ == '__main__':
    main()


