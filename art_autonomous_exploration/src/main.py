#!/usr/bin/env python

import rospy
import time

from speeds_assignment import RobotController

# The main function of the program
if __name__ == '__main__':

    # Wait for simulator and SLAM to initialize
    print "Waiting 5 seconds for initialization"
    time.sleep(5)
    
    # Initializes the ROS node
    rospy.init_node('robot_controller')
    # Creates a RobotController object
    rc = RobotController()
    # ROS waits for events
    rospy.spin()
