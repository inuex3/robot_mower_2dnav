#!/usr/bin/env python
import rospy
import csv
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf
import math

def odom_cb(data):
    f = open('output.csv', 'w')
    writer = csv.writer(f, lineterminator='\n')
    for pose in data.poses:
        data.header = data.header
        csvlist = []
        csvlist.append(str(pose.pose.position.x))
        csvlist.append(str(pose.pose.position.y))
        e = tf.transformations.euler_from_quaternion((pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w))
        csvlist.append(str(math.degrees(e[2])))
        print(csvlist)
        writer.writerow(csvlist)

rospy.init_node('path_node')

path_sub = rospy.Subscriber('/rtabmap/mapPath', Path, odom_cb)

if __name__ == '__main__':
    rospy.spin()
