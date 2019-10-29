#!/usr/bin/env python
# coding: utf-8

"""Display LIDAR data from specific node.

Attributes
----------
ANGLE_MAX : int
    maximum angle
ANGLE_MIN : int
    minimum angle
angles : list
    list of available angles
ctr : int
    counter of displayed data sets
intensities : list
    intensities from scan data
INTENSITY_MAX : int
    maximum intensity
INTENSITY_MIN : int
    minimum intensity
phist : plot
    histogram plot
pintensities : plot
    intensity plot
pranges : plot
    range plot
preflectivities : plot
    reflectivity plot
RANGE_MAX : int
    maximum range
RANGE_MIN : int
    minimum range
ranges : list
    ranges from scan data
reflectivities : list
    reflectivities from scan data
SETS : int
    number of displayed scans
TOPIC : str
    which topic should be listened to
"""

import rospy
from sensor_msgs.msg import LaserScan
import pyqtgraph as pg
import numpy as np
from turtlebot3_charge import calcReflectivities
from turtlebot3_charge import reorder
from turtlebot3_charge import getThreshold
from find_spots import THRESHOLD

# view settings
TOPIC = "scan_filtered"  # which topic should be listened to
SETS = 1  # number of displayed scans
ANGLE_MIN = -180  # min dispalyed angle
ANGLE_MAX = 180  # max displayed angle
RANGE_MIN = 0  # min displayed range
RANGE_MAX = 4  # max displayed range
INTENSITY_MIN = 0  # min displayed intensity
INTENSITY_MAX = 15000  # max displayed intensity

# global variables
ctr = 0  # counter for scans
angles = list(range(-180, 180))
ranges = list(np.zeros(360))
intensities = list(np.zeros(360))
reflectivities = list(np.zeros(360))

# setup windows
pg.setConfigOption("background", "w")
pg.setConfigOption("foreground", "k")

# setup ranges plot
pranges = pg.plot()
pranges.setTitle("Ranges")
pranges.setRange(xRange=range(ANGLE_MIN, ANGLE_MAX),
                 yRange=range(RANGE_MIN, RANGE_MAX))

# setup intensities plot
pintensities = pg.plot()
pintensities.setTitle("Intensities")
pintensities.setRange(xRange=range(ANGLE_MIN, ANGLE_MAX),
                      yRange=range(INTENSITY_MIN, INTENSITY_MAX))

# setup reflectivites plot
preflectivities = pg.plot()
preflectivities.setTitle("Reflectivities")
preflectivities.setRange(xRange=range(ANGLE_MIN, ANGLE_MAX),
                         yRange=range(10000))

# setup histogram plot
phist = pg.plot()
phist.setTitle("Reflectivity Histogram")
phist.setRange(xRange=range(10000), yRange=range(360))


def callback(data):
    """Is called when new sensor data is available.

    Parameters
    ----------
    data : LaserScan
            laser data from LIDAR
    """

    global angles
    global ranges
    global intensities
    global reflectivities

    # convert to lists
    intensities = list(data.intensities)
    ranges = list(data.ranges)

    # filter values
    for i in range(len(intensities)):
        if str(ranges[i]) == "nan" or ranges[i] <= 0.0:
            ranges[i] = -1.0
            intensities[i] = 0

    reflectivities = list(calcReflectivities(ranges, intensities))

    # reorder values
    intensities = reorder(intensities)
    ranges = reorder(ranges)
    reflectivities = reorder(reflectivities)


def update():
    """Update graph."""

    global ctr
    global angles
    global ranges
    global pranges
    global intensities
    global pintensities
    global reflectivities
    global preflectivities
    global phist

    # calc intensity histogram data
    rhist, redges = np.histogram(reflectivities)

    if ctr <= SETS:  # add another scan
        pintensities.plot(angles, intensities,
                          pen=None,
                          symbol="o",
                          symbolSize=5,
                          symbolBrush=(255, 0, 0, 255))
        pranges.plot(angles, ranges,
                     pen=None,
                     symbol="o",
                     symbolSize=5,
                     symbolBrush=(255, 0, 0, 255))
        preflectivities.plot(angles, reflectivities,
                             pen=None,
                             symbol="o",
                             symbolSize=5,
                             symbolBrush=(255, 0, 0, 255))
        phist.plot(redges[1:], rhist,
                   pen=None,
                   symbol="o",
                   symbolSize=5,
                   symbolBrush=(255, 0, 0, 255))
    else:  # clear graph and add scan
        pintensities.plot(angles, intensities,
                          clear=True,
                          pen=None,
                          symbol="o",
                          symbolSize=5,
                          symbolBrush=(255, 0, 0, 255))
        pranges.plot(angles, ranges,
                     clear=True,
                     pen=None,
                     symbol="o",
                     symbolSize=5,
                     symbolBrush=(255, 0, 0, 255))
        preflectivities.plot(angles, reflectivities,
                             clear=True,
                             pen=None,
                             symbol="o",
                             symbolBrush=(255, 0, 0, 255))
        phist.plot(redges[1:], rhist,
                   clear=True,
                   pen=None,
                   symbol="o",
                   symbolSize=5,
                   symbolBrush=(255, 0, 0, 255))
        ctr = 0

    # calc adaptive threholds
    intensities_threshold = getThreshold(intensities)
    reflectivities_threshold = getThreshold(reflectivities)

    rospy.logdebug("display_sensordata: \
        \n\tintensity threshold is {} \
        \n\treflectivity threshold is {}".format(intensities_threshold,
                                                 reflectivities_threshold))

    intensities_threshold_list = list()
    reflectivities_threshold_list = list()
    for a in angles:
        intensities_threshold_list.append(intensities_threshold)
        reflectivities_threshold_list.append(reflectivities_threshold)

    # draw red line if adaptive threshold is below const threshold
    if intensities_threshold >= THRESHOLD:
        pintensities.plot(angles, intensities_threshold_list,
                          symbol=None)
    else:
        pintensities.plot(angles, intensities_threshold_list,
                          symbol=None,
                          pen="r")

    if reflectivities_threshold >= THRESHOLD:
        preflectivities.plot(angles, reflectivities_threshold_list,
                             symbol=None)
    else:
        preflectivities.plot(angles, reflectivities_threshold_list,
                             symbol=None,
                             pen="r")

    ctr += 1
    pg.QtGui.QApplication.processEvents()


def listener():
    """Listen to a given ros topic."""

    rospy.init_node("display_sensordata", anonymous=True,
                    log_level=rospy.DEBUG)
    rospy.Subscriber(TOPIC, LaserScan, callback)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        update()
        rate.sleep()


def main():
    """Run listener."""

    listener()


if __name__ == "__main__":
    main()
