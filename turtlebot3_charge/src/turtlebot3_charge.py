#!/usr/bin/env python
# coding: utf-8

"""Project library.

Attributes
----------
ANGULAR_SPEED : float
    max turning speed
FORWARD_SPEED : float
    speed of robot when driving forward
HIST_OFFSET : int
    noise offset in histogram
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist

# global vars
FORWARD_SPEED = 0.1  # speed of robot when driving forward
ANGULAR_SPEED = 0.1  # max turning speed
HIST_OFFSET = 0  # noise offset in histogram


def calcReflectivities(_ranges, _intensities):
    """Calculate reflectivites from ranges and intensities.

    Parameters
    ----------
    _ranges : list
        ranges from LIDAR
    _intensities : list
        intensities from LIDAR

    Returns
    -------
    list
        reflectivities
    """

    reflectivities_ = list([])
    for i in range(len(_ranges)):
        rang = _ranges[i]
        inte = _intensities[i]

        # how to get reflectivity
        refl = rang * inte ** 2 / 10000

        reflectivities_.append(refl)
    return reflectivities_


def reorder(mylist):
    """Reorder list from 0 degree -> 360 degree to -180 degree -> 180 degree.

    Parameters
    ----------
    mylist : list
        old list

    Returns
    -------
    list
        new list
    """

    old_mylist = mylist
    mylist = old_mylist[180:]
    mylist += old_mylist[:180]
    return mylist


def cmd_vel(linear, angular):
    """Control movement of the robot.

    linear > 0 -> forward;
    linear < 0 -> backward;
    angular > 0 -> left;
    angular < 0 -> right

    Parameters
    ----------
    linear : float
        forward speed
    angular : float
        turning speed
    """

    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    cmd_pub.publish(twist)


def targetReached():
    """Run if target is reached."""

    global SUCCESS
    SUCCESS = True
    rospy.logdebug("targetReached")
    cmd_vel(0.0, 0.0)


def moveForward():
    """Move forward."""

    rospy.logdebug("moveForward")
    cmd_vel(FORWARD_SPEED, 0.0)


def moveBackward():
    """Move backward."""

    rospy.logdebug("moveBackward")
    cmd_vel(-FORWARD_SPEED, 0.0)


def turnLeft():
    """Turn left."""

    rospy.logdebug("turnLeft")
    cmd_vel(0.0, ANGULAR_SPEED)


def turnRight():
    """Turn right."""

    rospy.logdebug("turnRight")
    cmd_vel(0.0, -ANGULAR_SPEED)


def getMarksReflec(ranges, intensities, THRESHOLD):
    """Find marked spots due to reflectivity peaks.

    Parameters
    ----------
    ranges : list
        ranges from LIDAR
    intensities : list
        intensities from LIDAR
    THRESHOLD : float
        reflectivity threshold

    Returns
    -------
    list, list
        marks, marked_angles
    """

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

    return marks, marked_angles


def getThreshold(data):
    """Get threshold according to balanced histogram thresholding method.

    ( https://en.wikipedia.org/wiki/Balanced_histogram_thresholding )

    Parameters
    ----------
    data : list
        list of values

    Returns
    -------
    float
        threshold
    """

    def getIndex(entries, target):
        """Return index of an number in an array which is closest to a target.

        Parameters
        ----------
        entries : list
            elements which should be checked
        target : float
            target value

        Returns
        -------
        int
            index of element
        """
        distance = list()
        for e in entries:
            d = np.abs(target - e)
            distance.append(d)
            if d <= 0.0:
                break
        if len(entries) > 0:
            min_distance = np.min(distance)
            return distance.index(min_distance)
        else:
            return -1

    def getWeight(entries, start, end):
        """Get weight between start and end.

        Parameters
        ----------
        entries : list
            list of numbers
        start : int
            start index (inclusive)
        end : int
            end index (inclusive)

        Returns
        -------
        float
            weight
        """

        weight = 0.0
        for i in range(len(entries)):
            if i >= start or i <= end - 1:
                weight += i
        return weight

    n, e = np.histogram(data)

    I_s = e[0]
    I_e = e[-1]
    I_m = (I_s + I_e) / 2.0

    W_l = getWeight(n, getIndex(e, I_s), getIndex(e, I_m))
    W_r = getWeight(n, getIndex(e, I_m) + 1, getIndex(e, I_e))

    while W_r > W_l:
        W_r -= n[getIndex(e, I_e) + 1]
        I_e = e[getIndex(e, I_e) - 1]

        if (I_s + I_e) / 2.0 < I_m:
            W_l -= n[getIndex(n, I_m)]
            W_r += n[getIndex(n, I_m)]
            I_m -= 1.0

    return I_m


def getMarksAdap(intensities, THRESHOLD):
    """Find marked spots due to intensity histogram.

    Parameters
    ----------
    intensities : list
        intensities from LIDAR
    THRESHOLD : float
        intensity threshold

    Returns
    -------
    list, list
        marks, marked_angles
    """

    marks = list(np.zeros(360))  # 0 no spot detected / 1 spot detected
    marked_angles = list([])  # list of angles from detected spots

    # calc adaptive intensity threshold
    adaptive_intensity_threshold = getThreshold(intensities)

    # fallback
    if(adaptive_intensity_threshold <= THRESHOLD):
        adaptive_intensity_threshold = THRESHOLD

    rospy.logdebug("getTarget: adaptive threshold is " +
                   str(np.round(adaptive_intensity_threshold)))

    # save detected spots
    for i in range(len(intensities) - 1):
        if intensities[i] >= adaptive_intensity_threshold:
            marks[i] = 1
            marked_angles.append(i)

    return marks, marked_angles


def getMarksAdapReflec(ranges, intensities, THRESHOLD):
    """Find marked spots due to reflectivity peaks with an adaptive threshold.

    Parameters
    ----------
    ranges : list
        ranges from LIDAR
    intensities : list
        intensities from LIDAR
    THRESHOLD : float
        reflectivity threshold

    Returns
    -------
    list, list
        marks, marked_angles
    """

    # calculate reflectivities
    reflectivities = list(calcReflectivities(ranges, intensities))

    # filter invalid reflectivities
    for i in range(360):
        if reflectivities[i] <= 0.0:
            reflectivities[i] = 0.0

    refl_threshold = getThreshold(reflectivities)

    # fallback
    if refl_threshold <= THRESHOLD:
        refl_threshold = THRESHOLD

    rospy.logdebug("getTarget: adaptive threshold is " +
                   str(np.round(refl_threshold)))

    # detect reflective tape (for marks 0 = no; 1 = yes)
    # and save angles where it appears
    marks = list(np.zeros(360))
    marked_angles = list([])
    for i in range(360):
        if reflectivities[i] >= refl_threshold:
            marks[i] = 1
            marked_angles.append(i)

    return marks, marked_angles


def getTarget(ranges, intensities, THRESHOLD, REQ_ANGLES):
    """Evaluate target spot from LIDAR data.

    Parameters
    ----------
    ranges : list
        ranges from LIDAR
    intensities : list
        intensites from LIDAR
    THRESHOLD : float
        threshold
    REQ_ANGLES : int
        required angles

    Returns
    -------
    float or str, float or str
        target, targetRange
    """

    ranges = list(ranges)
    intensities = list(intensities)

    # filter values for invalid data
    for i in range(len(intensities)):
        if str(ranges[i]) == "nan" or ranges[i] <= 0.0:
            ranges[i] = -1.0
            intensities[i] = 0.0

    # marks, marked_angles = getMarksReflec(ranges, intensities, THRESHOLD)
    # marks, marked_angles = getMarksAdap(intensities, THRESHOLD)
    marks, marked_angles = getMarksAdapReflec(ranges, intensities, THRESHOLD)

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

    # calc distance to target
    targetRange = "NaN"
    if target != "NaN":
        targetRange = ranges[int(np.round(target))]

    rospy.logdebug("marks = " + str(marks))
    rospy.logdebug("marked_angles = " + str(marked_angles))

    return target, targetRange
