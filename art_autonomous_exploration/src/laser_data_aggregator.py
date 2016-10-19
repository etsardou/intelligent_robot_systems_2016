#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

# Class for reading the data from the laser sensor
class LaserDataAggregator:

    # Constructor
    def __init__(self):

        # Initialization of laser scan 
        self.laser_scan = []

        # ROS Subscribers to the robot's laser
        laser_topic = rospy.get_param("laser_topic")
        rospy.Subscriber(laser_topic, LaserScan, self.getDataLaser) 

    # Getting data from the laser
    def getDataLaser(self, data):

        # Get the measurements
        self.laser_scan = list(data.ranges)

        # Pay attention for special values
        for i in range(0, len(self.laser_scan)):
            if self.laser_scan[i] > data.range_max:
                self.laser_scan[i] = data.range_max
            elif self.laser_scan[i] < data.range_min:
                self.laser_scan[i] = data.range_min

