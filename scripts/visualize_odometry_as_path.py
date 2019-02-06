#!/usr/bin/env python
import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

path1 = Path()
path2 = Path()

def odom1_cb(data):
    global path1
    path1.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.header.frame_id = "map"
    pose.pose.position.x = data.pose.pose.position.x
    pose.pose.position.y = data.pose.pose.position.y
    path1.poses.append(pose)
    path1_pub.publish(path1)

def odom2_cb(data):
    global path2
    path2.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    path2.poses.append(pose)
    path2_pub.publish(path2)

rospy.init_node('path_node')
odom_sub = rospy.Subscriber('/base_pose_ground_truth', Odometry, odom1_cb)
odom2_sub = rospy.Subscriber('/odom', Odometry, odom2_cb)
path1_pub = rospy.Publisher('/true_path', Path, queue_size=10)
path2_pub = rospy.Publisher('/path', Path, queue_size=10)

if __name__ == '__main__':
    rospy.spin()
