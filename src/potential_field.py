#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Twist

rospy.init_node('potential_field')
class potential_field():
    def __init__(self):
        self.goal=[1,0] #목표지점
        self.fps=10
        self.goal_weight=1 
        self.obs_weight=1
        self.obs_flag=False

        rospy.Subscriber('/obs',PointCloud,self.obs_callback,queue_size=1)
        self.vel_pub=rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        rospy.Timer(1/self.fps,self.timer_callback)

    def pure_pursuit_point(self,point):
        heading = 0
        back_wheel=0.21
        WB=0.08
        g_dis = ((point[0]+back_wheel)**2+ point[1]**2)**(1/2)
        alpha = math.atan2(point[1], point[0] + back_wheel) - heading
        delta = math.atan2(2.0*WB*math.sin(alpha)/g_dis, 1.0)
        return delta

    def obs_callback(self,pointcloud):
        self.obs_flag=True
        self.obs=[[p.x , p.y] for p in pointcloud]

    def potential_obs(self,obs:list):
        def repulse(obs:list):
            obs_norm=(obs[0]**2+obs[1]**2)**0.5
            obs_normal_vector=[obs[0]/obs_norm,obs[1]/obs_norm]
            return [obs_normal_vector[0]*(-1)*self.obs_weight/obs_norm**2,obs_normal_vector[1]*self.obs_weight*(-1)/obs_norm**2]
        repulse_list=map(repulse,obs)
        x=0
        y=0
        for p in repulse_list:
            x=x+p[0]
            y=y+p[1]
        return [x,y]
    
    def potential_goal(self,obs:list):
        def repulse(obs:list):
            obs_norm=(obs[0]**2+obs[1]**2)**0.5
            obs_normal_vector=[obs[0]/obs_norm,obs[1]/obs_norm]
            return [obs_normal_vector[0]*self.goal_weight/obs_norm**2,obs_normal_vector[1]*self.goal_weight/obs_norm**2]
        attract_list=map(repulse,obs)
        x=0
        y=0
        for p in attract_list:
            x=x+p[0]
            y=y+p[1]
        return [x,y]
    


    def timer_callback(self,_event):
        if self.obs_flag==True:
            repulse=self.potential_obs(self.obs)
            attract=self.potential_goal([self.goal])
            print("repulse:",repulse)
            print("attract:",attract)
            local_waypoint=[repulse[0]+attract[0],repulse[1]+attract[1]]
            steer=self.pure_pursuit_point(local_waypoint) #radian
            
            




