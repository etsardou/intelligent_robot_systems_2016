#!/usr/bin/env python

import rospy
import random
import math
import time
import numpy as np
from timeit import default_timer as timer
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from _cpp_functions import ffi, lib

class Print:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    ORANGE = '\033[93m'
    RED = '\033[91m'
    END = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

    @staticmethod
    def art_print(txt, color):
      print color + str(txt) + Print.END

class Cffi:
    @staticmethod
    def brushfireFromObstacles(ogm, brush, ogml):
      #TODO improve: by map limits
      x = [np.array(v, dtype='int32') for v in ogm]
      xi = ffi.new(("int* [%d]") % (len(x)))
      for i in range(len(x)):
        xi[i] = ffi.cast("int *", x[i].ctypes.data)

      y = [np.array(v, dtype='int32') for v in brush]
      yi = ffi.new(("int* [%d]") % (len(y)))
      for i in range(len(y)):
        yi[i] = ffi.cast("int *", y[i].ctypes.data)

      br_c = lib.brushfireFromObstacles(xi, yi, len(x), len(x[0]), 
          ogml['min_x'], ogml['max_x'], ogml['min_y'], ogml['max_y'])
      # TODO: Must be faster!
      for i in range(ogm.shape[0]):
        for j in range(ogm.shape[1]):
          brush[i][j] = yi[i][j]

      return brush

    @staticmethod
    def thinning(skeleton, ogml):
      x = [np.array(v, dtype='int32') for v in skeleton]
      xi = ffi.new(("int* [%d]") % (len(x)))
      for i in range(len(x)):
        xi[i] = ffi.cast("int *", x[i].ctypes.data)

      y = [np.array(v, dtype='int32') for v in skeleton]
      yi = ffi.new(("int* [%d]") % (len(y)))
      for i in range(len(y)):
        yi[i] = ffi.cast("int *", y[i].ctypes.data)

      itime = time.time()
      br_c = lib.thinning(xi, yi, len(x), len(x[0]),
          ogml['min_x'], ogml['max_x'], ogml['min_y'], ogml['max_y'])
      Print.art_print("Pure skeletonization time: " + str(time.time() - itime), Print.BLUE)
      # TODO: Must be faster!
      itime = time.time()
      for i in range(skeleton.shape[0]):
        for j in range(skeleton.shape[1]):
          skeleton[i][j] = yi[i][j]
      Print.art_print("Skeletonization final copy: " + str(time.time() - itime), Print.BLUE)
      return skeleton

    @staticmethod
    def prune(skeleton, ogml, iterations):
      itime = time.time()
      x = [np.array(v, dtype='int32') for v in skeleton]
      xi = ffi.new(("int* [%d]") % (len(x)))
      for i in range(len(x)):
        xi[i] = ffi.cast("int *", x[i].ctypes.data)

      y = [np.array(v, dtype='int32') for v in skeleton]
      yi = ffi.new(("int* [%d]") % (len(y)))
      for i in range(len(y)):
        yi[i] = ffi.cast("int *", y[i].ctypes.data)

      br_c = lib.prune(xi, yi, len(x), len(x[0]),
          ogml['min_x'], ogml['max_x'], ogml['min_y'], ogml['max_y'], iterations)
      
      # TODO: Must be faster!
      for i in range(skeleton.shape[0]):
        for j in range(skeleton.shape[1]):
          skeleton[i][j] = yi[i][j]
      Print.art_print("Pruning time: " + str(time.time() - itime), Print.BLUE)
      return skeleton

class OgmOperations:

    @staticmethod
    def blurUnoccupiedOgm(ogm, ogml):
      local = np.copy(ogm)
      for i in range(ogml['min_x'], ogml['max_x']):
        for j in range(ogml['min_y'], ogml['max_y']):
          if ogm[i][j] > 49: # Not free
            c = 0
            for ii in range(-1, 2):
              for jj in range(-1, 2):
                if ogm[i + ii][j + jj] <= 49:
                  c += 1
            if c >= 4:
              local[i][j] = 0
      return local

    @staticmethod
    def findUsefulBoundaries(ogm, origin, resolution):
      min_x = origin['x'] / resolution
      min_y = origin['y'] / resolution
      max_x = origin['x'] / resolution
      max_y = origin['y'] / resolution
      
      x = ogm.shape[0]
      y = ogm.shape[1]

      # Search by x min
      ok = False
      for i in range(0, x, 20):
        for j in range(0, y):
          if ogm[i][j] != 51:
            min_x = i - 20
            ok = True
            break
        if ok:
          break

      # Search by x max
      ok = False
      for i in range(min_x + 20, x, 20):
        for j in range(0, y):
          if ogm[i][j] != 51:
            ok = True
            break
        if not ok:
          max_x = i
          break
        ok = False

      # Search by y min
      ok = False
      for j in range(0, y, 20):
        for i in range(0, x):
          if ogm[i][j] != 51:
            min_y = j - 20
            ok = True
            break
        if ok:
          break

      # Search by y max
      ok = False
      for j in range(min_y + 20, y, 20):
        for i in range(0, x):
          if ogm[i][j] != 51:
            ok = True
            break
        if not ok:
          max_y = j
          break
        ok = False

      RvizHandler.printMarker(\
            [
                [
                    min_x * resolution + origin['x'],
                    min_y * resolution + origin['y']
                ],
                [
                    max_x * resolution + origin['x'], 
                    min_y * resolution + origin['y']
                ],
                [
                    max_x * resolution + origin['x'], 
                    max_y * resolution + origin['y']
                ],
                [
                    min_x * resolution + origin['x'], 
                    max_y * resolution + origin['y']
                ]
            ],\
            1, # Type: Line strip
            0, # Action: Add
            "map", # Frame
            "art_ogm_boundary", # Namespace
            [0, 0.9, 0, 1.0], # Color RGBA
            0.2 # Scale
        )


      return {
          'min_x': min_x, 
          'max_x': max_x, 
          'min_y': min_y, 
          'max_y': max_y
          }

class RvizHandler:

    markers_publisher = \
            rospy.Publisher('art_rviz_markers', MarkerArray, queue_size = 10)

    # Poses: list of coordinates
    # m_type: http://wiki.ros.org/rviz/DisplayTypes/Marker
    # action: 0 = add/modify, 1 = (deprecated), 2 = delete, New in Indigo 3 = deleteall
    # frame: Usually map
    # ns: whatever
    # color: [r,g,b,a]
    # scale: [sx,sy,sz]
    @staticmethod
    def printMarker(poses, m_type, action, frame, ns, color, scale):

        # Publish the targets for visualization purposes
        markers = MarkerArray()
        c = 0
        for s in poses:
            #print "Print " + str(s) + " for ns = " + ns
            c += 1
            st = Marker()
            st.header.frame_id = frame
            st.ns = ns
            st.id = c
            st.header.stamp = rospy.Time(0)
            st.type = m_type
            st.action = action

            st.pose.position.x = s[0]
            st.pose.position.y = s[1]

            st.color.r = color[0]
            st.color.g = color[1]
            st.color.b = color[2]
            st.color.a = color[3]

            st.scale.x = scale
            st.scale.y = scale
            st.scale.z = scale
            markers.markers.append(st)

            if ns == 'art_topological_nodes':
              c += 1
              st = Marker()
              st.header.frame_id = frame
              st.ns = ns
              st.id = c
              st.header.stamp = rospy.Time(0)
              st.type = 9
              st.text = str((int) (c/2 - 1))
              st.action = action

              st.pose.position.x = s[0] + 0.15
              st.pose.position.y = s[1] + 0.15
              st.pose.position.z = 0.2

              st.color.r = 0
              st.color.g = 0
              st.color.b = 0
              st.color.a = 0.8

              st.scale.x = scale * 4
              st.scale.y = scale * 4
              st.scale.z = scale * 4
              markers.markers.append(st)


        RvizHandler.markers_publisher.publish(markers)

