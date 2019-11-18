#!/usr/bin/env python
import rospy
import tf
import numpy as np
from rtabmap_ros.srv import ResetPose
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import *
from sensor_msgs.msg import NavSatFix
import message_filters

start = Odometry()
goal = PoseStamped()
pub = rospy.Publisher('/gnss_odom_from_start', Odometry, queue_size=1)
position_pub = rospy.Publisher('/rtabmap/global_pose', Odometry, queue_size=1)

def callback_goal(G):
    global goal
    goal = G

def callback_start(Odom):
    global start
    start = Odom

def callback(Odom):
    global start
    global goal
    gnss_odom = Odometry()
    gnss_odom = Odom
    robot_position = PoseWithCovarianceStamped()
    start.header.frame_id = "map"
    gnss_odom.header.frame_id = "map"
    gnss_odom.pose.pose.position.x = (Odom.pose.pose.position.x - start.pose.pose.position.x)
    gnss_odom.pose.pose.position.y = (Odom.pose.pose.position.y - start.pose.pose.position.y)
    gnss_odom.pose.pose.position.z = 0.0
    gnss_odom.pose.pose.orientation = Odom.pose.pose.orientation
    robot_position.header = Odom.header
    robot_position.header.frame_id = "map"
    robot_position.pose.pose = goal.pose
    robot_position.pose.covariance = [0.001, 0, 0, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 9999, 0, 0, 0, 0, 0, 0, 9999, 0, 0, 0, 0, 0, 0, 9999]
    if abs(gnss_odom.pose.pose.position.x) < 1000:
        pub.publish(gnss_odom)
        if goal.header.seq < 1:
            print(robot_position)
            position_pub.publish(robot_position)

def listen():
    start_sub = rospy.Subscriber('/set_start', Odometry, callback_start)
    rospy.sleep(3)
    odom_sub = rospy.Subscriber('/gnss_odom', Odometry, callback)
    goal_sub = rospy.Subscriber('/move_base/current_goal', PoseStamped, callback_goal)
    rospy.spin()

def main():
    rospy.init_node('gnss_odom_from_zero')
    listen()

if __name__ == '__main__':
    main()


