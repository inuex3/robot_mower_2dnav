#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import String
from mavros.utils import *
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

pub = rospy.Publisher('imu/data', Imu, queue_size=10)

def callback_imu(Imu):
    Imu.orientation.y, Imu.angular_velocity.y, Imu.linear_acceleration.y = -Imu.orientation.y, -Imu.angular_velocity.y, -Imu.linear_acceleration.y
    Imu.orientation.z, Imu.angular_velocity.z, Imu.linear_acceleration.z = Imu.orientation.z, Imu.angular_velocity.z, Imu.linear_acceleration.z
    print(Imu)
    pub.publish(Imu)
    
def listen():
    rospy.Subscriber('/mavros/imu/data', Imu, callback_imu)
    rospy.spin()

def main():
    rospy.init_node('subscriber_imu_body')
    listen()

if __name__ == '__main__':
    main()


