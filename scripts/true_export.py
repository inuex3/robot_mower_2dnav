#!/usr/bin/env python
import rospy
import csv
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf
import math

true_path = Path()

def odom_true_cb(data):
    global true_path
    true_path = data

def odom_cb(data):
    if data.header.stamp.secs < 1566367845:#1566367845ouhuku2:#1566368551:forward
        print(data.header.stamp.secs)
        global true_path
        f_t = open('true_output.csv', 'w')
        writer_t = csv.writer(f_t, lineterminator='\n')
        for true in data.poses:
            csvlist_true = []
            csvlist_true.append(str(true.pose.position.x))
            csvlist_true.append(str(true.pose.position.y))
            e = tf.transformations.euler_from_quaternion((true.pose.orientation.x, true.pose.orientation.y, true.pose.orientation.z, true.pose.orientation.w))
            csvlist_true.append(str(math.degrees(e[2])))
            writer_t.writerow(csvlist_true)

rospy.init_node('path_node')

path_sub = rospy.Subscriber('/true_path', Path, odom_cb)
true_path_sub = rospy.Subscriber('/true_path', Path, odom_true_cb)

if __name__ == '__main__':
    rospy.spin()
