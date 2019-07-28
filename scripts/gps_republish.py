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
    pub.publish(Gps)    

def listen():
    rospy.Subscriber('/RTK_NavSatFix', NavSatFix, callback_gps)
    rospy.spin()

def main():
    rospy.init_node('subscriber_gps')
    listen()

if __name__ == '__main__':
    main()


