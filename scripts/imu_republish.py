#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import String
from mavros.utils import *
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

pub = rospy.Publisher('/imu/data', Imu, queue_size=10)

def callback_imu(Imu):
    e = tf.transformations.euler_from_quaternion((Imu.orientation.x,Imu.orientation.y,Imu.orientation.z,Imu.orientation.w))
    Imu.orientation.x,Imu.orientation.y,Imu.orientation.z,Imu.orientation.w = tf.transformations.quaternion_from_euler(-e[0], e[1], e[2])
    Imu.angular_velocity.x, Imu.linear_acceleration.y = -Imu.angular_velocity.x, -Imu.linear_acceleration.y
    #print(Imu)
    pub.publish(Imu)
    
def listen():
    rospy.Subscriber('/mavros/imu/data', Imu, callback_imu)
    rospy.spin()

def main():
    rospy.init_node('subscriber_imu')
    listen()

if __name__ == '__main__':
    main()


