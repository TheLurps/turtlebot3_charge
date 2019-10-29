#!/usr/bin/env python
# coding: utf-8

"""Publish location of charging station.

Attributes
----------
my_markers : list
    list of spots
target : Marker
    target marker
"""

import rospy
from visualization_msgs.msg import Marker

my_markers = list()
target = Marker()
target.header.stamp.secs = 0


def clean_up(markers):
    """Reduce number of spot marker.

    Parameters
    ----------
    markers : list
        old list

    Returns
    -------
    list
        new list
    """

    num_markers = 500
    return markers[-num_markers:]


def new_spot(marker):
    """Call when new spot is published.

    Parameters
    ----------
    marker : Marker
        new marker obj
    """

    global my_markers
    global target

    rospy.logdebug("\n find_charging_station: new marker received!")
    my_markers.append(marker)
    my_markers = clean_up(my_markers)

    target_x = 0.0
    target_y = 0.0

    for marker in my_markers:
        target_x += marker.pose.position.x
        target_y += marker.pose.position.y

    my_markers_len = len(my_markers)
    if my_markers_len >= 1:
        target_x /= my_markers_len
        target_y /= my_markers_len

    target.id = 0
    target.type = target.CYLINDER
    target.scale.x = 0.05
    target.scale.y = 0.05
    target.scale.z = 1.0
    target.pose.position.x = target_x
    target.pose.position.y = target_y
    target.header = marker.header
    target.header.stamp = rospy.Time.now()
    target.color.r = 0.0
    target.color.g = 255.0
    target.color.b = 0.0
    target.color.a = 255.0
    target.lifetime.secs = 300


def listener():
    """Startup node and listen to some others."""

    global LISTEN_TO
    global target

    rospy.init_node("find_charging_station",
                    anonymous=True, log_level=rospy.DEBUG)
    rospy.Subscriber("spots_transformed", Marker, new_spot)
    pub = rospy.Publisher("cstation", Marker, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        if target.header.stamp.secs > 0:
            pub.publish(target)
        rate.sleep()


def main():
    """Run listener."""

    listener()


if __name__ == "__main__":
    main()
