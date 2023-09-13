#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from sensor_msgs.msg import LaserScan, Imu, PointCloud
from geometry_msgs.msg import Point32
rospy.init_node("data_hub")

class data_hub():
    def __init__(self):
        rospy.Subscriber('/Imu',Imu,self.imu_callback,queue_size=1)
        rospy.Subscriber('/scan',LaserScan,self.lidar_callback,queue_size=1)
        self.obs_pub=rospy.Publisher('/obs',PointCloud,queue_size=1)
        self.obs_xy=[]
        self.imu=[]

    
    def imu_callback(self,ros_msg):
        pass


    def laser_callback(self,scan_msg):
        pc_msg = PointCloud()
        pc_msg.header = scan_msg.header

        for i, r in enumerate(scan_msg.ranges):
            if not math.isinf(r):
                point = Point32()
                point.x = r * math.cos(scan_msg.angle_min + i * scan_msg.angle_increment)
                point.y = r * math.sin(scan_msg.angle_min + i * scan_msg.angle_increment)
                point.z = 0.0  # Assuming a 2D laser scanner

                pc_msg.points.append(point)

        self.obs_pub.publish(pc_msg)


        
        







