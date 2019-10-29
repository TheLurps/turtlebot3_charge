#!/usr/bin/env python
# coding: utf-8

"""Write battery voltage to csv.

Attributes
----------
LISTEN_TO : str
    ros topic with battery states
ofile : file obj
    csv file
writer : write obj
    write to specified file
"""

import rospy
import time
import csv
from sensor_msgs.msg import BatteryState

# global vars
LISTEN_TO = "battery_states"  # specify ros topic which should be listened

ofile = open("battery_states.csv", "wb")
writer = csv.writer(ofile)


def callback(data):
    """Is called when new data is available.

    Parameters
    ----------
    data : BatteryState
        BatteryState from node
    """

    voltage = data.voltage
    now = time.time()

    writer.writerow([now, voltage])

    rospy.logdebug("%s @ %s" % (voltage, now))


def listener():
    """Listen to a given ros topic."""

    global LISTEN_TO
    rospy.init_node("battery_status", anonymous=True, log_level=rospy.DEBUG)
    rospy.Subscriber(LISTEN_TO, BatteryState, callback)
    rospy.spin()  # keep listening


def main():
    """Run listener."""

    listener()


if __name__ == "__main__":
    main()
