#!/usr/bin/env python

import rospy
import math
import time
from utilities import Print

from ogmpp_communications.srv import OgmppPathPlanningSrv
from ogmpp_communications.srv import OgmppSetMapSrv
from ogmpp_communications.srv import OgmppPathPlanningSrvRequest
from ogmpp_communications.srv import OgmppPathPlanningSrvResponse
from ogmpp_communications.srv import OgmppSetMapSrvRequest

from geometry_msgs.msg import Point

# Class that implements path planning via A*
class PathPlanning:

    # Constructor
    def __init__(self):
        self.ogmpp_pp = rospy.ServiceProxy('/ogmpp_path_planners/plan', OgmppPathPlanningSrv)
        self.ogmpp_map = rospy.ServiceProxy('/ogmpp_path_planners/set_map', OgmppSetMapSrv)

    def setMap(self, ogm):
        # Set the map
        map_req = OgmppSetMapSrvRequest()
        map_req.map = ogm
        self.ogmpp_map(map_req)

    # Function to be called from navigation
    def createPath(self, robot_pose, target_pose, resolution):

        # Ask for path
        resp = OgmppPathPlanningSrvResponse()
        req = OgmppPathPlanningSrvRequest()
        req.data.begin = Point()
        req.data.end = Point()

        req.data.begin.x = robot_pose[0] * resolution
        req.data.begin.y = robot_pose[1] * resolution
        req.data.end.x = target_pose[0] * resolution
        req.data.end.y = target_pose[1] * resolution
        req.method = "uniform_prm"

        tinit = time.time()
        resp = self.ogmpp_pp(req)
        Print.art_print("Path planning time: " + str(time.time() - tinit), Print.ORANGE)

        path = []
        for p in resp.path.poses:
            path.append([p.pose.position.x / resolution, \
                p.pose.position.y / resolution])

        return path

