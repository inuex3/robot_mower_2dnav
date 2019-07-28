#!/usr/bin/env python
import rospy
import tf

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

pub = rospy.Publisher('/imu/data', Imu, queue_size=10)

def callback_imu(Imu_data):
    new_imu = Imu()
    new_imu.header = Imu_data.header
    new_imu.orientation = Imu_data.orientation
    new_imu.angular_velocity = Imu_data.angular_velocity
    new_imu.linear_acceleration = Imu_data.linear_acceleration
    new_imu.orientation_covariance[0], new_imu.orientation_covariance[4], new_imu.orientation_covariance[8] = 0.00000001, 0.00000001, 0.00000001
    new_imu.angular_velocity_covariance[0], new_imu.angular_velocity_covariance[4], new_imu.angular_velocity_covariance[8] = 0.00000001, 0.00000001, 0.00000001
    new_imu.linear_acceleration_covariance[0], new_imu.linear_acceleration_covariance[4], new_imu.linear_acceleration_covariance[8] = 0.00000001, 0.00000001, 0.00000001
    pub.publish(new_imu)
    
def listen():
    rospy.Subscriber('/gx5/imu/data', Imu, callback_imu)
    rospy.spin()

def main():
    rospy.init_node('subscriber_imu')
    listen()

if __name__ == '__main__':
    main()


