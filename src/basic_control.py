#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

rospy.init_node("basic_control")

cmd_pub=rospy.Publisher("cmd_vel",Twist,queue_size=1)

twist=Twist()
twist.linear.x=1
twist.angular.z=1


while not rospy.is_shutdown():
    cmd_pub.publish(twist)