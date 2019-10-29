#!/usr/bin/env python
# coding: utf-8

"""Find marked spots and pub as marker.

Attributes
----------
ANGLE_THRESHOLD : float
    max acceptable angle offset of target
LISTEN_TO : str
    specify ros topic which should be listened to
PUB_TO : str
    specify ros topic which should be published to
RANGE_THRESHOLD : float
    min range between robot and target
REQ_ANGLES : int
    number of angles which should be at least recognized
THRESHOLD : int
    threshold to recognize reflective tape
"""

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from turtlebot3_charge import getTarget

# global vars
LISTEN_TO = "scan_filtered"  # specify ros topic which should be listened to
PUB_TO = "spots"  # specify ros topic which should be published to
THRESHOLD = 3000  # threshold to recognize reflective tape
REQ_ANGLES = 2  # number of angles which should be at least recognized
ANGLE_THRESHOLD = 10.0  # max acceptable angle offset of target
RANGE_THRESHOLD = 0.19  # min range between robot and target


def pubSpot(scan, target, targetRange, FRAME_ID):
    """Generate spot scan data from input.

    Parameters
    ----------
    scan : LaserScan
        scan from LIDAR
    target : float or str
        target angle in degree
    targetRange : float
        distance to target in m
    FRAME_ID : str
        frame id
    """

    output = LaserScan()

    output.header.seq = scan.header.seq
    output.header.stamp.secs = scan.header.stamp.secs
    output.header.stamp.nsecs = scan.header.stamp.nsecs
    output.header.frame_id = FRAME_ID
    output.angle_min = scan.angle_min
    output.angle_max = scan.angle_max
    output.angle_increment = scan.angle_increment
    output.time_increment = scan.time_increment
    output.scan_time = scan.scan_time
    output.range_min = scan.range_min
    output.range_max = scan.range_max
    output.ranges = np.zeros(360)
    output.intensities = np.zeros(360)

    if target != "NaN":
        output.ranges[int(target)] = targetRange
        output.intensities[int(target)] = 10000.0

    pub = rospy.Publisher("spots_laserdata", LaserScan, queue_size=1)
    pub.publish(output)

    spot = Marker()
    spot.id = np.random.random_integers(2 ** 16 - 1)
    spot.type = spot.CYLINDER
    spot.scale.x = 0.05
    spot.scale.y = 0.05
    spot.scale.z = 1.0
    spot.pose.position.x = targetRange * np.cos(target / 360.0 * 2.0 * np.pi)
    spot.pose.position.y = targetRange * np.sin(target / 360.0 * 2.0 * np.pi)
    spot.header.frame_id = FRAME_ID
    spot.header.stamp = rospy.Time.now()
    spot.color.r = 255.0
    spot.color.g = 255.0
    spot.color.b = 255.0
    spot.color.a = 255.0
    spot.lifetime.secs = 300

    pub = rospy.Publisher(PUB_TO, Marker, queue_size=1)
    pub.publish(spot)


def callback(data):
    """Is called when new sensor data is available.

    Parameters
    ----------
    data : LaserScan
        scan from LIDAR
    """

    global THRESHOLD
    global REQ_ANGLES
    global PUB_TO

    rospy.logdebug("\n\nfind_spots: new callback")

    target, targetRange = getTarget(
        data.ranges, data.intensities, THRESHOLD, REQ_ANGLES)

    # publish target spot if one was found
    if target != "NaN":
        debug_msg = "target: " + \
            str(np.round(target, 1)) + "° @ " + \
            str(np.round(targetRange, 4)) + "m"
        rospy.logdebug(debug_msg)

        # publish spot
        pubSpot(data, target, targetRange, data.header.frame_id)


def listener():
    """Listen to a given ros topic."""

    global LISTEN_TO
    rospy.init_node("find_spots", anonymous=True, log_level=rospy.DEBUG)
    rospy.Subscriber(LISTEN_TO, LaserScan, callback)
    rospy.spin()  # keep listening


def main():
    """Run listener."""

    listener()


if __name__ == "__main__":
    main()
