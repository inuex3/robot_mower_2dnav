#!/usr/bin/env python
import rospy
import csv
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf
import math

csvlist = []

def odom_cb(data):
    if data.header.stamp.secs < 1566367845:#1566367845ouhuku2:#1566368551:forward
        f = open('output.csv', 'w')
        writer = csv.writer(f, lineterminator='\n')
        global csvlist
        csvlist.append(rospy.get_time())
        writer.writerow(csvlist)

rospy.init_node('path_node')

path_sub = rospy.Subscriber('/rtabmap/mapPath', Path, odom_cb)

if __name__ == '__main__':
    rospy.spin()
