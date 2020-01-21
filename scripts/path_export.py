#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

f = open('path.txt','a')

from_time = 1566367515.781358003
end_time =  1566367846.78736066

def callback_gps(Gps):
    if from_time - 0.25 < rospy.get_time() < end_time:
        print("          " + str(Gps.longitude) + "," + str(Gps.latitude) + ",0")
        f.write("          " + str(Gps.longitude) + "," + str(Gps.latitude) + "0\n")

def listen():
    rospy.Subscriber('/RTK_NavSatFix', NavSatFix, callback_gps)
    rospy.spin()

def main():
    rospy.init_node('subscriber_gps')
    listen()

if __name__ == '__main__':
    main()


