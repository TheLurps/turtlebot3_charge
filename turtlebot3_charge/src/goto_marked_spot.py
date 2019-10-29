#!/usr/bin/env python
# coding: utf-8

"""Drive to marked spot from last scan data.

Attributes
----------
ANGLE_THRESHOLD : float
    max acceptable angle offset of target
LISTEN_TO : str
    specify ros topic which should be listened to
RANGE_THRESHOLD : float
    min range between robot and target
REQ_ANGLES : int
    number of angles which should be at least recognized
SUCCESS : bool
    is target reached? then True
THRESHOLD : int
    reflectivity threshold to recognize reflective tape
"""

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from turtlebot3_charge import calcReflectivities
from turtlebot3_charge import cmd_vel
from turtlebot3_charge import turnLeft
from turtlebot3_charge import turnRight
from turtlebot3_charge import moveForward
from turtlebot3_charge import targetReached

# global vars
LISTEN_TO = "scan_filtered"  # specify ros topic which should be listened to
THRESHOLD = 2000  # reflectivity threshold to recognize reflective tape
REQ_ANGLES = 2  # number of angles which should be at least recognized
ANGLE_THRESHOLD = 10.0  # max acceptable angle offset of target
RANGE_THRESHOLD = 0.19  # min range between robot and target
SUCCESS = False  # is target reached? then True


def callback(data):
    """Is called when new sensor data is available.

    Parameters
    ----------
    data : LaserScan
        scan from LIDAR
    """

    global SUCCESS

    # received data
    intensities = list(data.intensities)
    ranges = list(data.ranges)

    # filter values for invalid data
    for i in range(len(intensities)):
        if str(ranges[i]) == "nan" or ranges[i] <= 0.0:
            ranges[i] = -1.0
            intensities[i] = -10000

    # calculate reflectivities
    reflectivities = list(calcReflectivities(ranges, intensities))

    # filter invalid reflectivities
    for i in range(360):
        if reflectivities[i] <= 0.0:
            reflectivities[i] = 0.0

    # get average reflectivity value
    avg_refl = np.average(reflectivities)

    # detect reflective tape (for marks 0 = no; 1 = yes)
    # and save angles where it appears
    marks = list(np.zeros(360))
    marked_angles = list([])
    for i in range(360):
        if reflectivities[i] >= avg_refl + THRESHOLD:
            marks[i] = 1
            marked_angles.append(i)

    # determine target angle
    target = "NaN"
    if len(marked_angles) > 0:

        # for a target angle around 0° we have to modify our data set a little
        # to calc the avg in the next step
        if marks[0] != 0:
            for i in range(len(marked_angles)):
                if marked_angles[i] < 180:
                    marked_angles[i] *= -1

        target = np.round(np.average(marked_angles))

    # check if enough angles where recognized
    if len(marked_angles) < REQ_ANGLES:
        target = "NaN"

    # get range from sensor to nearest obj
    min_range = min([r for r in ranges if r >= 0.0])

    # print debug msg
    rospy.logdebug("%s \t from %s angles \t min_range: %s"
                   % (target, len(marked_angles), min_range))

    # determine movement
    if SUCCESS:  # target reached
        rospy.logdebug("SUCCESS!")
    else:  # target not reached
        if target != "NaN":  # target known
            # move forward
            if target <= ANGLE_THRESHOLD \
                    or target - 360.0 > ANGLE_THRESHOLD * -1:
                if min_range <= RANGE_THRESHOLD:
                    targetReached()
                else:
                    moveForward()
            # turn left
            elif target <= 180.0:
                turnLeft()
            # turn right
            elif target >= 180.0:
                turnRight()
        else:
            # do nothing
            cmd_vel(0.0, 0.0)


def listener():
    """Listen to a given ros topic."""

    global LISTEN_TO
    rospy.init_node("goto_marked_spot", anonymous=True, log_level=rospy.DEBUG)
    rospy.Subscriber(LISTEN_TO, LaserScan, callback)
    rospy.spin()  # keep listening


def main():
    """Run listener."""

    listener()


if __name__ == "__main__":
    main()
