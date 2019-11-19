#!/usr/bin/env python
import rospy
import tf
import numpy as np
from tf.transformations import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rtabmap_ros.srv import ResetPose
from geometry_msgs.msg import PointStamped
import math

rospy.init_node('set_pose')
start = Odometry()
pose_pub = rospy.Publisher('/rtabmap/global_pose', PoseWithCovarianceStamped, queue_size=1)
start_pub = rospy.Publisher('/set_start', Odometry, queue_size=1)
pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=1)
reset_pose = rospy.ServiceProxy('/rtabmap/reset_odom_to_pose', ResetPose)
point = PointStamped()
tf_br = tf.TransformBroadcaster()
tf_listener = tf.TransformListener()

def print_obstacle_and_landmark_position(x, y, heading):
    obstacle_position = rospy.get_param('/obstacle/obstacle_position')
    point.header.frame_id = "map"
    for i, p in enumerate(obstacle_position): 
        point.header.stamp = rospy.Time.now()
        tf_br.sendTransform((p[0], p[1], 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "obstacle_position" + str(i), "odom")
        tf_listener.waitForTransform("/odom", "/" + "obstacle_position" + str(i), rospy.Time(0), rospy.Duration(0.1))
        point_position = tf_listener.lookupTransform("/map", "/" + "obstacle_position" + str(i), rospy.Time(0))
        #pub.publish(point)
        point.point.x = point_position[0][0] * math.cos(heading) - point_position[0][1]*math.sin(heading) + x
        point.point.y = point_position[0][0] * math.cos(heading) + point_position[0][1]*math.sin(heading) + y
        print(point)
        rospy.sleep(0.1)

def publish_goal(heading):
    rospy.sleep(0.1)
    point.header.frame_id = "map"
    goal_point = rospy.get_param('/obstacle/goal_point')
    heading = heading - math.pi/2
    for i, p in enumerate(goal_point): 
        point.header.stamp = rospy.Time.now()
        tf_br.sendTransform((p[0], p[1], 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "point" + str(i), "odom")
        tf_listener.waitForTransform("/odom", "/" + "point" + str(i), rospy.Time(0), rospy.Duration(0.1))
        point_position = tf_listener.lookupTransform("/map", "/" + "point" + str(i), rospy.Time(0))
        point.point.x = point_position[0][0]# * math.cos(heading) - point_position[0][1]*math.sin(heading)
        point.point.y = point_position[0][1]# * math.sin(heading) + point_position[0][1]*math.cos(heading)
        point.point.z = 0.0
        pub.publish(point)
        rospy.sleep(0.1)

def callback_odom(Odom):
    global n
    global start
    gnss_odom = Odometry()
    posi = PoseWithCovarianceStamped()
    start = Odom
    start.header.frame_id = "map"
    e = tf.transformations.euler_from_quaternion((start.pose.pose.orientation.x, start.pose.pose.orientation.y, start.pose.pose.orientation.z, start.pose.pose.orientation.w))
    heading = math.atan(e[2]) - math.pi/2
    #reset_pose(0.0, 0.0, 0.0, 0, 0, heading)
    posi.header.stamp = rospy.Time.now()
    start_pub.publish(start)
    subscriber.unregister()
    rospy.sleep(1)
    print_obstacle_and_landmark_position(start.pose.pose.position.x, start.pose.pose.position.y, heading)
    raw_input("\nPress Enter to publish point...")
    publish_goal(heading)
    rospy.signal_shutdown('Quit')

subscriber = rospy.Subscriber('/gnss_odom', Odometry, callback_odom)

def listen():
    rospy.spin()

def main():
    rospy.sleep(1)
    listen()

if __name__ == '__main__':
    main()


