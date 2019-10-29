#!/usr/bin/env python
# coding: utf-8

"""Drive to charging station.

Attributes
----------
DISTANCE_FINAL : float
    final distance to charging station
DISTANCE_MIN : float
    distance that is reachable with move_base
goal : PoseStamped
    target destination
move_base_success : bool
    was move_base successful?
move_final_success : bool
    final destination reached?
position : PoseStamped
    actual position of the robot
"""

import rospy
import tf
import numpy as np
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from sensor_msgs.msg import BatteryState
from turtlebot3_charge import turnLeft
from turtlebot3_charge import turnRight
from turtlebot3_charge import moveForward
from turtlebot3_charge import cmd_vel
from turtlebot3_charge import targetReached

DISTANCE_MIN = 0.5
DISTANCE_FINAL = 0.142
DISTANCE_BLOCKING = 0.17

goal = PoseStamped()
move_base_success = False
move_final_success = False
position = PoseStamped()


def new_cstation(m):
    """Is called when new position of the charging station is received.

    Parameters
    ----------
    m : Marker
        marker with position data of charging station
    """

    global DISTANCE_MIN
    global goal
    global position

    def get_distance(x1, y1, x2, y2):
        """Calc distance of two point with 2D cartesian coordinates.

        Parameters
        ----------
        x1 : float
            x coord of point 1
        y1 : float
            y coord of point 1
        x2 : float
            x coord of point 2
        y2 : float
            y coord of point 2

        Returns
        -------
        float
            distance
        """
        d = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return d

    # get coords
    x_target = m.pose.position.x
    y_target = m.pose.position.y
    x_self = position.pose.position.x
    y_self = position.pose.position.y

    # calc angle between robot and target
    yaw = np.arctan2((y_target - y_self), (x_target - x_self))

    # print("goto_charging_station: yaw from position to target is \
    #    {}".format(180.0 * yaw / np.pi))

    q = tf.transformations.quaternion_from_euler(0, 0, yaw, "ryxz")

    # needed distance from robot to target destination
    distance = get_distance(x_self, y_self, x_target,
                            y_target) - DISTANCE_MIN

    goal.header = m.header
    goal.pose.position.x = x_self + distance * np.cos(yaw)
    goal.pose.position.y = y_self + distance * np.sin(yaw)
    goal.pose.position.z = 0.0
    goal.pose.orientation.x = q[0]
    goal.pose.orientation.y = q[1]
    goal.pose.orientation.z = q[2]
    goal.pose.orientation.w = q[3]


# def findEdge(ranges):
#     def nextValue(values, index):
#         for i in range(index + 1, len(values)):
#             if values[i] != "nan":
#                 return values[i]
#         return -1

#     def prevValue(values, index):
#         indices = range(index - 1, len(values)).sort(reverse=True)
#         for i in indices:
#             if values[i] != "nan":
#                 return values[i]
#         return -1

#     ranges = list(ranges)

#     for i in range(len(ranges)):
#         if ranges[i] == "nan":
#             if i >= 0 and i <= len(ranges):
#                 ranges[i] = (prevValue(ranges, i) + nextValue(ranges, i)) \
#                     / 2.0
#             elif i <= 0:
#                 ranges[i] = nextValue(ranges, i) \
#                     - (nextValue(ranges, i + 1) - nextValue(ranges, i))
#             elif i >= len(ranges):
#                 ranges[i] = prevValue(ranges, i) \
#                     - (prevValue(ranges, i - 1) - prevValue(ranges, i))


def new_laserscan(scan):
    """Is called when new laser data is received.

    Parameters
    ----------
    scan : LaserScan
        new laser data
    """

    global DISTANCE_FINAL
    global DISTANCE_BLOCKING
    global move_base_success
    global move_final_success

    # movement after move_base is successful
    if move_base_success and not move_final_success:
        angle = scan.intensities.index(max(scan.intensities))
        angle_threshold = 3

        if scan.ranges[59] <= DISTANCE_BLOCKING:
            rospy.logdebug(
                "goto_charging_station: something is block the robot")
            cmd_vel(-0.1, 0.2)
        elif scan.ranges[-60] <= DISTANCE_BLOCKING:
            rospy.logdebug(
                "goto_charging_station: something is block the robot")
            cmd_vel(-0.1, -0.2)
        elif angle <= 180:
            # print("angle below 180")
            if angle - angle_threshold <= 0:
                for r in scan.ranges[:5] + scan.ranges[-5:]:
                    reached = False
                    if r <= DISTANCE_FINAL \
                            or r == "nan":
                        reached = True
                        break
                if not reached:
                    moveForward()
                else:
                    targetReached()
                    move_final_success = True
                    rospy.logdebug("\ngoto_charging_station: \
                        final target reached!")
            else:
                turnLeft()
        else:
            # print("angle above 180")
            if angle + angle_threshold >= 360:
                for r in scan.ranges[:5] + scan.ranges[-5:]:
                    reached = False
                    if r <= DISTANCE_FINAL \
                            or r == "nan":
                        reached = True
                        break
                if not reached:
                    moveForward()
                else:
                    targetReached()
                    move_final_success = True
                    rospy.logdebug("\ngoto_charging_station: \
                        final target reached!")
            else:
                turnRight()


def new_position(_position):
    """Is called when new robot position is received.

    Parameters
    ----------
    _position : PoseStamped
        new position
    """

    global position
    position = _position


def new_goalstatus(goals):
    """Is called when new goal status is received.

    Parameters
    ----------
    goals : GoalStatusArray
        array of goal status
    """

    global move_base_success
    global goal

    if not move_base_success:
        if len(goals.status_list) > 0:
            # is last goal successful and is it the last that was published
            if goals.status_list[-1].status == 3 and \
                    goals.status_list[-1].goal_id.stamp.secs \
                    == goal.header.stamp.secs:
                move_base_success = True
                rospy.logdebug("\ngoto_charging_station: \
                    move_base target reached!")


def new_battery_state(state):
    """Is called when new battery state is received.

    Parameters
    ----------
    state : BatteryState
        new battery state
    """

    global move_final_success
    if state.power_supply_status == "1":
        targetReached()
        move_final_success = True


def listener():
    """Listen to a given ros topic."""

    global move_base_success
    global move_final_success
    global goal

    rospy.init_node("goto_marked_spot", anonymous=True, log_level=rospy.DEBUG)
    rospy.Subscriber("cstation", Marker, new_cstation)
    rospy.Subscriber("scan_filtered", LaserScan, new_laserscan)
    rospy.Subscriber("pose_publisher", PoseStamped, new_position)
    rospy.Subscriber("move_base/status", GoalStatusArray, new_goalstatus)
    rospy.Subscriber("battery_states", BatteryState, new_battery_state)

    pub_goal = rospy.Publisher("move_base_simple/goal",
                               PoseStamped,
                               queue_size=1)

    targetReached()

    old_goal = goal
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        if not move_base_success and not move_final_success \
                and not old_goal == goal:
            # print("\ngoto_charging_station: new move_base target")
            old_goal = goal
            pub_goal.publish(goal)
            rate.sleep()


def main():
    """Run listener."""

    listener()


if __name__ == "__main__":
    main()
