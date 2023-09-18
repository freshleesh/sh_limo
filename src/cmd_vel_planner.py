#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

class cmd_vel_planner():
    def __init__(self):
        self.target_speed = 0
        self.target_steer = 0
        self.now_speed = 0
        self.now_steer = 0
        self.speed_gap = 0.1
        self.steer_gap = 0.1

    def callback(self,msg):
        self.target_speed = msg.linear.x
        self.target_steer = msg.angular.z
        
    def chase(self):
        if self.now_speed < self.target_speed:
            self.now_speed += self.speed_gap
        
        elif self.now_speed > self.target_speed:
            self.now_speed -= self.speed_gap

        if self.now_steer < self.target_steer:
            self.now_steer += self.steer_gap
        
        elif self.now_steer > self.target_steer:
            self.now_steer -= self.steer_gap


if __name__ == 'main':
    cvp = cmd_vel_planner()
    rospy.init_node('cmd_vel_planner')
    rospy.Subscriber('limo_vel',Twist,cvp.callback,queue_size=1)
    vel_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
    rate=rospy.Rate(10)
    cvp.chase()
    t = Twist()
    t.linear.x = cvp.now_speed
    t.angular.z = cvp.now_steer
    vel_pub.publish(t)
