#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TransformStamped, Vector3Stamped
from std_msgs.msg import Int16, Bool
from tf.transformations import quaternion_multiply, quaternion_from_euler, quaternion_inverse, euler_from_quaternion
import tf2_geometry_msgs
import tf2_ros
from math import floor

import geometry_msgs.msg
def odom_callback(msg):
    # Extracting relevant data from the Odometry message
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    broadcaster = tf2_ros.TransformBroadcaster()

    # Creating a TransformStamped message
    odom_trans = TransformStamped()

    # Setting the header frame ID and timestamp
    odom_trans.header.stamp = rospy.Time.now()
    odom_trans.header.frame_id = "odom_ticks"
    odom_trans.child_frame_id = "base_link"  # Adjust this frame ID according to your setup

    # Setting the translation
    odom_trans.transform.translation.x = position.x
    odom_trans.transform.translation.y = position.y
    odom_trans.transform.translation.z = position.z

    # Setting the orientation
    odom_trans.transform.rotation.x = orientation.x
    odom_trans.transform.rotation.y = orientation.y
    odom_trans.transform.rotation.z = orientation.z
    odom_trans.transform.rotation.w = orientation.w

    # Publishing the TF transform
    broadcaster.sendTransform(odom_trans)

if __name__ == '__main__':
    rospy.init_node('wheel_odom_transform_publisher')

    # Initialize a TF broadcaster


    # Subscribe to the Odometry topic
    rospy.Subscriber("/odom_ticks", Odometry, odom_callback)

    # Spin ROS
    rospy.spin()


