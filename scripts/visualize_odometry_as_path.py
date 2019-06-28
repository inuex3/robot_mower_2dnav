#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

path1 = Path()
path2 = Path()
path3 = Path()
path4 = Path()
num = 0
num2 = 0
num3 = 0


def odom1_cb(data):
    global path1
    path1.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.header.frame_id = 'map'
    pose.pose.position.x = data.pose.pose.position.x
    pose.pose.position.y = data.pose.pose.position.y
    path1.poses.append(pose)
    path1_pub.publish(path1)

def odom2_cb(data):
    global path2
    global num
    data.pose.pose.position.x = data.pose.pose.position.x + 2.0
    data.pose.pose.position.y = data.pose.pose.position.y + 2.0
    if num % 10 == 0:
        path2.header = data.header
        pose = PoseStamped()
        pose.header = data.header
        pose.pose = data.pose.pose
        path2.poses.append(pose)
        path2_pub.publish(path2)
    num = num + 1

def odom3_cb(data):
    global path3
    global num2
    if num2 % 10 == 0:
        path3.header = data.header
        pose = PoseStamped()
        pose.header = data.header
        pose.pose = data.pose.pose
        path3.poses.append(pose)
        path3_pub.publish(path3)
    num2 = num2 + 1

def odom4_cb(data):
    global path4
    global num3
    if num3 % 10 == 0:
        path4.header = data.header
        pose = PoseStamped()
        pose.header = data.header
        pose.pose = data.pose.pose
        path4.poses.append(pose)
        path4_pub.publish(path3)
    num3 = num3 + 1

rospy.init_node('path_node')
odom_sub = rospy.Subscriber('/base_pose_ground_truth', Odometry, odom1_cb)
odom2_sub = rospy.Subscriber('/odom', Odometry, odom2_cb)
odom3_sub = rospy.Subscriber('/vo', Odometry, odom3_cb)
odom4_sub = rospy.Subscriber('/vo2', Odometry, odom4_cb)
odom_pub = rospy.Publisher('/odom_0.7', Path, queue_size=1)
path1_pub = rospy.Publisher('/true_path', Path, queue_size=1)
path2_pub = rospy.Publisher('/path', Path, queue_size=1)
path3_pub = rospy.Publisher('/vo_path', Path, queue_size=1)
path4_pub = rospy.Publisher('/vo1_path', Path, queue_size=1)

if __name__ == '__main__':
    rospy.spin()
