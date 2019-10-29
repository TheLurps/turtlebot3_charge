#!/usr/bin/env python
# coding: utf-8

"""Print intensities from LIDAR scan."""

import rospy
from sensor_msgs.msg import LaserScan


def callback(data):
    """Is called when new sensor data is available.

    Parameters
    ----------
    data : LaserScan
        scan from LIDAR
    """

    intensities = data.intensities
    ranges = data.ranges

    # filter intensities
    for i in range(len(intensities)):
        if ranges[i] == "nan":
            intensities[i] = 0.0

    rospy.logdebug(intensities)


def listener():
    """Listen to a given ros topic."""

    rospy.init_node("listener", anonymous=True, log_level=rospy.DEBUG)
    rospy.Subscriber("/scan_filtered", LaserScan, callback)
    rospy.spin()


if __name__ == "__main__":
    listener()
