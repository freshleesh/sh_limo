#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, select, tty, termios
import rospy   
from std_msgs.msg import String

if __name__=='__main__':
    rospy.init_node("key_read")
    key_pub = rospy.Publisher("keys",String,queue_size=1)
    rate = rospy.Rate(100)
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    while not rospy.is_shutdown():
        if select.select([sys.stdin],[],[],0)[0] == [sys.stdin]:
            key_pub.publish(sys.stdin.read(1))
            print("input:",sys.stdin.read(1))
        rate.sleep()
    termios.tcsetattr(sys.stdin,termios.TCSADRAIN,old_attr)

    