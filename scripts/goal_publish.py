#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray

start = Odometry()
status = GoalStatusArray()

def callback_status(st):
    global n
    global status
    status = st

def callback_odom(Odom):
    global n
    global start
    start = Odom

if __name__ == '__main__':
    rospy.init_node('patrol')
    x = 367949.045549
    y = 3955737.19975
    listener = tf.TransformListener()
    subscriber = rospy.Subscriber('/set_start', Odometry, callback_odom)
    subscriber = rospy.Subscriber('/move_base/status', GoalStatusArray, callback_status)
    goal_pub = rospy.Publisher('/goal', Int32, queue_size=1)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    client.wait_for_server()
    rospy.sleep(1)
    listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(4.0))
    rospy.sleep(1)
    global start
    global status
    raw_input("\nPress Enter to publish point...")
    origin_x = x - start.pose.pose.position.x
    origin_y = y - start.pose.pose.position.y
    while True:
        i = 0
        while True:
            goal_pose = MoveBaseGoal()
            goal_pose.target_pose.header.seq = i
            goal_pose.target_pose.header.stamp = rospy.Time.now()
            goal_pose.target_pose.header.frame_id = 'map'
            if (0.2 + 0.4 * int(i/2))>8:
                print("finished")
                rospy.signal_shutdown('Quit')
            if i % 4 == 0:
                goal_pose.target_pose.pose.position.x = 0 + origin_x
                goal_pose.target_pose.pose.position.y = 0.2 + 0.4 * int(i/2) + origin_y
            if i % 4 == 1:
                goal_pose.target_pose.pose.position.x = -16 + origin_x
                goal_pose.target_pose.pose.position.y = 0.2 + 0.4 * int(i/2) + origin_y
            if i % 4 == 2:
                goal_pose.target_pose.pose.position.x = -16 + origin_x
                goal_pose.target_pose.pose.position.y = 0.2 + 0.4 * int(i/2) + origin_y
            if i % 4 == 3:
                goal_pose.target_pose.pose.position.x = 0 + origin_x
                goal_pose.target_pose.pose.position.y = 0.2 + 0.4 * int(i/2) + origin_y
            goal_pose.target_pose.pose.position.z = 0
            q = tf.transformations.quaternion_from_euler(0, 0, 3.1415)
            goal_pose.target_pose.pose.orientation.x = q[0]
            goal_pose.target_pose.pose.orientation.y = q[1]
            goal_pose.target_pose.pose.orientation.z = q[2]
            goal_pose.target_pose.pose.orientation.w = q[3]
            client.send_goal(goal_pose)
            goal_pub.publish(i)
            rospy.sleep(0.2)
            if i > 0:
                print(int(i/2) != i/2)
                listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(0.1))
                position = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
                if ((position[0][0] - goal_pose.target_pose.pose.position.x) * (position[0][0] - goal_pose.target_pose.pose.position.x) + (position[0][1] - goal_pose.target_pose.pose.position.y) * (position[0][1] - goal_pose.target_pose.pose.position.y)) <1.0:
                    client.cancel_goal()
                    i = i + 1
                    continue
            i = i + 1
            while True:
                rospy.sleep(0.2)
                listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(0.1))
                position = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
                if ((position[0][0] - goal_pose.target_pose.pose.position.x) * (position[0][0] - goal_pose.target_pose.pose.position.x) + (position[0][1] - goal_pose.target_pose.pose.position.y) * (position[0][1] - goal_pose.target_pose.pose.position.y)) <0.09:
                    print "next!!"
                    break
                else:
                    rospy.sleep(0.5)
