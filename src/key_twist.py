#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

key_mapping={ 'w' : [1, 0], 'a' : [0, 1], 's' : [0, 0], 'd' : [0, -1]}

def key_callback(msg): 
    vel = key_mapping[msg.data[0]]
    t = Twist()
    t.linear.x = vel[0]
    t.angular.z = vel[1]
    cmd_pub.publish(t)


if __name__ == '__main__':
    rospy.init_node("key_twist")
    rospy.Subscriber("/keys", String, key_callback, queue_size=1)
    cmd_pub = rospy.Publisher("/limo_vel", Twist, queue_size=1)
    rospy.spin()