#!/usr/bin/env python
# coding: utf-8

"""Transform spot marker to map frame.

Attributes
----------
listener : TransformListener
    TF TransformListener
"""

import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

listener = tf.TransformListener()


def new_spot(marker):
    """Is called when new spot is published.

    Parameters
    ----------
    marker : Marker
        new marker obj
    """
    global listener

    listener.waitForTransform(marker.header.frame_id, "/map",
                              rospy.Time(0), rospy.Duration(2.0))
    old_pose = PoseStamped()

    old_pose.pose = marker.pose
    old_pose.header.frame_id = marker.header.frame_id
    new_pose = listener.transformPose("/map", old_pose)

    marker.header = new_pose.header
    marker.pose = new_pose.pose

    pub = rospy.Publisher("spots_transformed", Marker, queue_size=1)
    pub.publish(marker)
    rospy.logdebug(
        "\n\ntransform_spots: new spot transformed \n{}".format(marker))


def main():
    """Run listener."""

    rospy.init_node("transform_spots", anonymous=True, log_level=rospy.DEBUG)
    rospy.Subscriber("spots", Marker, new_spot)
    rospy.spin()


if __name__ == "__main__":
    main()
