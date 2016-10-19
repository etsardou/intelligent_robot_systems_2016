#!/usr/bin/env python

import rospy
import random
import math
import numpy
from timeit import default_timer as timer
from utilities import RvizHandler
from utilities import Cffi
from sets import Set
import scipy.misc

#scipy.misc.imsave('/home/manos/Desktop/test.png', self.coverage)

class Brushfires:

    def obstaclesBrushfireCffi(self, ogm, ogm_limits):
        brush = numpy.full(ogm.shape, -1)

        width = ogm.shape[0]
        height = ogm.shape[1]

        for i in range(ogm_limits['min_x'], ogm_limits['max_x'] - 1):
            for j in range(ogm_limits['min_y'], ogm_limits['max_y'] - 1):
                if ogm[i][j] > 49 or ogm[i][j] == -1:
                    brush[i][j] = 0

        brush = Cffi.brushfireFromObstacles(ogm, brush, ogm_limits)
        return brush


    def obstaclesBrushfire(self, ogm, ogm_limits):
        brush = numpy.full(ogm.shape, -1)

        width = ogm.shape[0]
        height = ogm.shape[1]

        _current = []

        for i in range(ogm_limits['min_x'], ogm_limits['max_x'] - 1):
            for j in range(ogm_limits['min_y'], ogm_limits['max_y'] - 1):
                if ogm[i][j] > 49 or ogm[i][j] == -1:
                    brush[i][j] = 0
                    _current.append((i, j))

        _next = []

        expanded = True
        counter = -1
        while expanded:
            expanded = False
            counter += 1
            for c in _current:
                lim = False
                for kx in range(-1, 2):
                    for ky in range(-1, 2):
                        if ky == 0 and kx == 0:
                            continue
                        _x = c[0] + kx
                        _y = c[1] + ky
                        if brush[_x][_y] == -1 and ogm[_x][_y] < 49:
                            brush[_x][_y] = counter + 1
                            _next.append((_x, _y))
                            expanded = True
            _current = _next
            _next = []

        return brush

    def coverageLimitsBrushfire(self, ogm, coverage, robot_pose, origin, resolution):

        limits = Set()

        width = ogm.shape[0]
        height = ogm.shape[1]

        for i in range(1, width - 1):
            for j in range(1, height - 1):
                if coverage[i][j] == 100:
                    ogm_ok = True
                    for ii in range(-1, 2):
                        for jj in range(-1, 2):
                            if ogm[i + ii][j + jj] > 49:
                                ogm_ok = False
                    if ogm_ok is False:
                        continue
                    cov_ok = False
                    for ii in range(-1, 2):
                        for jj in range(-1, 2):
                            if coverage[i + ii][j + jj] != 100:
                                cov_ok = True
                    if cov_ok:
                        limits.add((\
                                float(i) * resolution + origin['x'], \
                                float(j) * resolution + origin['y']\
                                ))
        return limits

    def closestUncoveredBrushfire(self, ogm, coverage, brushogm, robot_pose, origin, resolution):
        limits = Set()

        brush = numpy.full(ogm.shape, -1)

        width = ogm.shape[0]
        height = ogm.shape[1]

        r_x = int(robot_pose['x_px'] - origin['x'] / resolution + 0.5)
        r_y = int(robot_pose['y_px'] - origin['y'] / resolution + 0.5)

        brush[r_x][r_y] = 0

        _current = []
        _current.append((r_x, r_y))
        _next = []

        expanded = True
        counter = -1
        while expanded:
            expanded = False
            counter += 1
            for c in _current:
                lim = False
                for kx in range(-1, 2):
                    for ky in range(-1, 2):
                        if ky == 0 and kx == 0:
                            continue
                        _x = c[0] + kx
                        _y = c[1] + ky
                        if brush[_x][_y] == -1 and coverage[_x][_y] != 0:
                            brush[_x][_y] = counter + 1
                            _next.append((_x, _y))
                            expanded = True
                        if coverage[_x][_y] == 0:
                            lim = True
                        if ogm[_x][_y] > 49:
                            lim = False
                if lim == True and brushogm[c[0]][c[1]] > 5: #TODO add this in param
                    return\
                            [c[0] * resolution + origin['x'], \
                            c[1] * resolution + origin['y']]
            _current = _next
            _next = []

        return limits
