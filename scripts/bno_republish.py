#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu

pub = rospy.Publisher('/imu/data_output', Imu, queue_size=1)

def callback_imu(Imu):
    Imu.angular_velocity_covariance = [0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001]
    Imu.linear_acceleration_covariance = [0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001]
    pub.publish(Imu)
    
def listen():
    rospy.Subscriber('/imu/data_out', Imu, callback_imu)
    rospy.spin()

def main():
    rospy.init_node('subscriber_imu')
    listen()

if __name__ == '__main__':
    main()


