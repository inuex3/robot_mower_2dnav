#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import PointStamped

rospy.init_node('publish_point')
pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=1)
point = PointStamped()
tf_br = tf.TransformBroadcaster()
tf_listener = tf.TransformListener()

def callback(Odom):
    global n
    global start
    gnss_odom = Odometry()
    posi = PoseWithCovarianceStamped()
    start = Odom
    start.header.frame_id = "map"
    e = tf.transformations.euler_from_quaternion((start.pose.pose.orientation.x, start.pose.pose.orientation.y, start.pose.pose.orientation.z, start.pose.pose.orientation.w))
    reset_pose(0.0, 0.0, 0.0, 0, 0,e[2])
    posi.header.stamp = rospy.Time.now()
    posi.pose.pose.position.x = 0.0
    posi.pose.pose.position.y = 0.0
    posi.header.frame_id = "base_link"
    posi.pose.pose.orientation = start.pose.pose.orientation
    posi.pose.covariance = [0.001, 0, 0, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 9999, 0, 0, 0, 0, 0, 0, 9999, 0, 0, 0, 0, 0, 0, 9999]
    time_before = rospy.get_time()
    pose_pub.publish(posi)
    start_pub.publish(start)
    subscriber.unregister()
    rospy.signal_shutdown('Quit')

def publish_goal():
    rospy.sleep(8)
    point.header.frame_id = "map"
    goal_point = rospy.get_param('~goal_point')
    for i, p in enumerate(goal_point): 
        point.header.stamp = rospy.Time.now()
        tf_br.sendTransform((p[0], p[1], 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "point" + str(i), "odom")
        tf_listener.waitForTransform("/odom", "/" + "point" + str(i), rospy.Time(0), rospy.Duration(0.1))
        point_position = tf_listener.lookupTransform("/map", "/" + "point" + str(i), rospy.Time(0))
        point.point.x = point_position[0][0]
        point.point.y = point_position[0][1]
        point.point.z = 0.0
        #pub.publish(point)
        rospy.sleep(0.01)

def main():
    subscriber = rospy.Subscriber('/gnss_odom', Odometry, callback_odom)
    

if __name__ == '__main__':
    main()



