#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from sensor_msgs.msg import LaserScan, Imu, PointCloud
from geometry_msgs.msg import Point32

rospy.init_node("data_hub")

class data_hub():
    def __init__(self):
        self.fps=10 #초당 처리 횟수
        rospy.Subscriber('/Imu',Imu,self.imu_callback,queue_size=1)
        rospy.Subscriber('/scan',LaserScan,self.laser_callback,queue_size=1)
        rospy.Timer(rospy.Duration(1.0/self.fps), self.timerCallback)
        self.obs_pub=rospy.Publisher('/obs',PointCloud,queue_size=1)
        self.imu=[]
        self.laser_flag=False
        self.delta=0.1

    def imu_callback(self,ros_msg):
        pass

    def laser_callback(self,scan_msg):
        self.laser_flag=True
        self.obs_xy = PointCloud()
        self.obs_xy.header.frame_id='map'
        self.obs_xy.header = scan_msg.header

        temp_points=[]
        for i, r in enumerate(scan_msg.ranges):
            if not math.isinf(r):
                x = r * math.cos(scan_msg.angle_min + i * scan_msg.angle_increment)
                y = r * math.sin(scan_msg.angle_min + i * scan_msg.angle_increment)

                #voxelize
                x = (x//self.delta)*self.delta
                y = (y//self.delta)*self.delta
                temp_points.append([x,y])

        obs_xy=list(set(temp_points))
        for p in obs_xy:
            point=Point32()
            point.x=p[0]
            point.y=p[1]
            point.z=0
            self.obs_xy.points.append(point)

    def timerCallback(self, _event):
        if self.laser_flag == True:
            self.obs_pub.publish(self.obs_xy)

new_class=data_hub()
while not rospy.is_shutdown():
    rospy.spin()
        
        







