#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

pub = rospy.Publisher('/rtabmap/gps/fix', NavSatFix, queue_size=1)

def callback_gps(Gps):
    Gps.header.frame_id = "base_link"
    if Gps.position_covariance[0] < 0.5 and Gps.status.status == 1:
        Gps.altitude = 0
        Gps.position_covariance = [Gps.position_covariance[0] * Gps.position_covariance[0]/9, 0.0, 0.0, 0.0, Gps.position_covariance[4] * Gps.position_covariance[4]/9, 0.0, 0.0, 0.0, Gps.position_covariance[8] * Gps.position_covariance[8]/9]
        pub.publish(Gps)
    elif Gps.status.status == 2:
        Gps.position_covariance = [Gps.position_covariance[0] * Gps.position_covariance[0]/9, 0.0, 0.0, 0.0, Gps.position_covariance[4] * Gps.position_covariance[4]/9, 0.0, 0.0, 0.0, Gps.position_covariance[8] * Gps.position_covariance[8]/9]
        pub.publish(Gps)    
def listen():
    rospy.Subscriber('/RTK_NavSatFix', NavSatFix, callback_gps)
    rospy.spin()

def main():
    rospy.init_node('subscriber_gps')
    listen()

if __name__ == '__main__':
    main()


