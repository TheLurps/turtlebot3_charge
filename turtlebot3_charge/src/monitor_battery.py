#!/usr/bin/env python
# coding: utf-8

"""Monitor battery status and start goto charging station routine when needed.

Attributes
----------
BAD_LAUNCH_FILE : string
    launch file with drive to charging station behavior
bad_start : bool
    should drive to charging station behavior be started?
bad_started : bool
    is drive to charging station behavior started?
bad_stop : bool
    should drive to charging station behavior be stopped?
GOOD_LAUNCH_FILE : string
    launch file with normal operation
good_start : bool
    should normal operation be started?
good_started : bool
    is normal operation started?
good_stop : bool
    should normal operation be stopped?
voltage_good : bool
    is voltage high enough for normal operation?
VOLTAGE_REF : float
    voltage below that will invoke drive to charging station
VOLTAGE_TARGET : int
    voltage above that will continue operation
"""

import rospy
from sensor_msgs.msg import BatteryState
import roslaunch
import numpy as np

# below that will invoke drive to charging station
VOLTAGE_REF = 11.2

# above that will continue operation
VOLTAGE_TARGET = 12

# relative path would be nice here
GOOD_LAUNCH_FILE = ["/home/psawaffle1/" +
                    "catkin_ws/src/turtlebot3_charge/" +
                    "turtlebot3_charge/launch/move_random.launch"]
BAD_LAUNCH_FILE = ["/home/psawaffle1/" +
                   "catkin_ws/src/turtlebot3_charge/" +
                   "turtlebot3_charge/launch/goto_charging_station.launch"]

voltage_good = True
good_started = False
bad_started = False
good_start = False
bad_start = False
good_stop = False
bad_stop = False


def new_battery_state(state):
    """Is called when new battery state is received.

    Parameters
    ----------
    state : BatteryState
        new battery state
    """

    global VOLTAGE_REF
    global VOLTAGE_TARGET

    global voltage_good
    global good_started
    global bad_started
    global good_start
    global bad_start
    global good_stop
    global bad_stop

    voltage = np.round(state.voltage, 1)

    # check wheter battery voltage is good or bad
    if voltage <= VOLTAGE_REF:
        voltage_good = False
    elif voltage >= VOLTAGE_TARGET:
        voltage_good = True

    if voltage_good:
        rospy.logdebug(
            "monitor_battery: voltage is good with {}".format(state.voltage))

        if bad_started:
            bad_started = False
            bad_stop = True
        if not good_started:
            good_started = True
            good_start = True
    else:
        rospy.logdebug(
            "monitor_battery: voltage is bad with {}".format(state.voltage))

        if good_started:
            good_started = False
            good_stop = True
        if not bad_started:
            bad_started = True
            bad_start = True


def listener():
    """Listen to a given ros topic."""

    global GOOD_LAUNCH_FILE
    global BAD_LAUNCH_FILE
    global good_start
    global bad_start
    global good_stop
    global bad_stop

    rospy.init_node("monitor_battery", anonymous=True, log_level=rospy.DEBUG)
    rospy.Subscriber("battery_state", BatteryState, new_battery_state)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    good = roslaunch.parent.ROSLaunchParent(uuid, GOOD_LAUNCH_FILE)
    bad = roslaunch.parent.ROSLaunchParent(uuid, BAD_LAUNCH_FILE)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        if good_start:
            good_start = False
            good.start()
        elif good_stop:
            good_stop = False
            good.shutdown()
        elif bad_start:
            bad_start = False
            bad.start()
        elif bad_stop:
            bad_stop = False
            bad.shutdown()

        rate.sleep()


def main():
    """Run listener."""

    listener()


if __name__ == "__main__":
    main()
