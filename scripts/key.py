#!/usr/bin/python
# -*- coding:utf-8 -*-

import rospy
import sys
import select
import termios
import time

from geometry_msgs.msg import Twist

def teleop():
    linear_value = 1.0
    angular_value = 1.0

    while not rospy.is_shutdown():
        kb = input()
        twist = Twist()
    
        if str(kb) == '0':
            print("exit")
            exit()

        elif str(kb) == "w":    # forward
            twist.linear.x = linear_value; twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
            pub.publish(twist)

        elif str(kb) == "a":    # left
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = angular_value;
            pub.publish(twist)

        elif str(kb) == "s":    # backward
            twist.linear.x = -linear_value; twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
            pub.publish(twist)

        elif str(kb) == "d":    # right
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = -angular_value;
            pub.publish(twist)

        elif str(kb) == "e":    # right forward
            twist.linear.x = linear_value; twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = -angular_value;
            pub.publish(twist)

        elif str(kb) == "q":    # left forward
            twist.linear.x = linear_value; twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = angular_value;
            pub.publish(twist)

        elif str(kb) == "z":    # left backward
            twist.linear.x = -linear_value; twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = angular_value;
            pub.publish(twist)

        elif str(kb) == "c":    # right backward
            twist.linear.x = -linear_value; twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = -angular_value;
            pub.publish(twist)

        elif str(kb) == "i":    # linear speed increase
            linear_value = linear_value + 0.05
        elif str(kb) == "o":    # linear speed decrease
            linear_value = linear_value - 0.05
        elif str(kb) == "k":    # angular speed increase
            angular_value = angular_value + 0.05
        elif str(kb) == "l":    # angular speed decrease
            angular_value = angular_value - 0.05


        elif kb == None:
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
            pub.publish(twist)       

        #time.sleep(0.0)

def input():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] &= ~termios.ICANON
    new[3] &= ~termios.ECHO

    try:
        termios.tcsetattr(fd, termios.TCSANOW, new)
        i,o,e = select.select([sys.stdin],[],[], 0.1)
        if (i):
            key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, old)

    if (i):
        return key
    else:
        return None   

if __name__=="__main__":
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    rospy.init_node('teleop_twist_keyboard')
    manual_str = "ready to sanyo_keyboard_teleop\n\rw:forward, a:left, s:backward, d:right\n\re:r_forward, q:l_forward, z:l_backward, c:r_backward\n\ri:linear speed increase, o:decrease\n\rk:angular speed increase, l:decrease\n\r0:exit"

    print manual_str

    teleop()
