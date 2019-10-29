#!/usr/bin/env python
# coding: utf-8

"""Move around and avoid obstacles."""

import rospy
from sensor_msgs.msg import LaserScan
from turtlebot3_charge import turnRight
from turtlebot3_charge import moveForward
from turtlebot3_charge import moveBackward


def new_laserscan(scan):
    """Is called when new laser data is received.

    Parameters
    ----------
    scan : LaserScan
        new laser data
    """

    def obstacle(ranges, min_distance, start_angle, end_angle):
        """Check if there is an obstacle.

        Parameters
        ----------
        ranges : tuple
            ranges to check
        min_distance : float
            minimal distance to obstacle
        start_angle : int
            first angle to check
        end_angle : int
            last angle to check
        """

        angles = range(360) + range(360)
        start_index = angles.index(start_angle - 1)
        end_index = angles.index(end_angle - 1)
        angles = angles[start_index: end_index]

        for i in angles:
            if ranges[i] <= min_distance and not ranges[i] == "nan":
                return True

        return False

    # in that tolerance obstacles should be avoided (degree)
    tolerance = 30

    # minimal distance to obstacles
    distance_min = 0.6

    collision = False
    recovery = False

    for i in range(tolerance):
        if scan.ranges[i] <= distance_min or \
                scan.ranges[359 - i] <= distance_min:
            collision = True
        if scan.ranges[i] <= distance_min * 0.5 or \
                scan.ranges[359 - i] <= distance_min * 0.5:
            recovery = True
            break

    if recovery:
        moveBackward()
    elif collision and not recovery:
        turnRight()
    else:
        moveForward()


def listener():
    """Listen to a given ros topic."""

    rospy.init_node("move_random", anonymous=True, log_level=rospy.DEBUG)
    rospy.Subscriber("scan_filtered", LaserScan, new_laserscan)

    rospy.spin()


def main():
    """Run listener."""

    listener()


if __name__ == "__main__":
    main()
