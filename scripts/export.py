#!/usr/bin/env python
import rospy
import csv
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf
import math

#from_time = 1566368048.486150979
#end_time = 1566368551.485430955
from_time = 1566367515.781358003
end_time =1566367846.78736066
n = 0
f = open('test.csv','a')
mx = 0

def odom_cb(gnss_odom):
    global n, mx
    if from_time < rospy.get_time() <end_time:
        if n == 0:
            writer = csv.writer(f)
            writer.writerow(["Time","x","y","theta","x_gnss","y_gnss","theta_gnss"])
        n = 1
        prev = gnss_odom
        try:
            now = rospy.Time.now()
            listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(1))
            (trans,rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
            e = tf.transformations.euler_from_quaternion((rot[0],rot[1],rot[2],rot[3]))
            e_gnss = tf.transformations.euler_from_quaternion((gnss_odom.pose.pose.orientation.x, gnss_odom.pose.pose.orientation.y, gnss_odom.pose.pose.orientation.z, gnss_odom.pose.pose.orientation.w))
            writer = csv.writer(f)
            if mx < ((gnss_odom.pose.pose.position.x - trans[0])**2 + (gnss_odom.pose.pose.position.y - trans[1])**2)**0.5:
                mx = ((gnss_odom.pose.pose.position.x - trans[0])**2 + (gnss_odom.pose.pose.position.y - trans[1])**2)**0.5
                print(mx)
            writer.writerow([rospy.get_time(),trans[0],trans[1],e[2],gnss_odom.pose.pose.position.x,gnss_odom.pose.pose.position.y,e_gnss[2] + 0.045])
        except Exception as e:
            print(e)


def path_cb(data):
    if end_time - 2.0 < rospy.get_time() < end_time:#1566367845ouhuku2:#1566368551:forward
        f_path = open('test_path.csv', 'w')
        writer = csv.writer(f_path, lineterminator='\n')
        for pose in data.poses:
            e_path = tf.transformations.euler_from_quaternion((pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w))
            writer.writerow([pose.pose.position.x, pose.pose.position.y, e_path[2]])

path_sub = rospy.Subscriber('/rtabmap/mapPath', Path, path_cb)

rospy.init_node('path_node')
listener = tf.TransformListener()
path_sub = rospy.Subscriber('/gnss_odom_from_start', Odometry, odom_cb)

if __name__ == '__main__':
    rospy.spin()
