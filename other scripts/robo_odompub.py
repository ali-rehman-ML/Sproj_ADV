#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Vector3
from robomaster import robot
import numpy as np
import time
from math import atan2, radians

 
def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

class odom_publisher:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.robot=robot.Robot()
        self.robot.initialize(conn_type="ap")
        self.chassis=self.robot.chassis
        self.chassis.sub_position(freq=10, callback=self.sub_position_handler)
        self.chassis.sub_attitude(freq=10, callback=self.sub_attitude_info_handler)
        

        # Initialize ROS node and publisher
        rospy.init_node('odometry_publisher')
        self.odom_pub = rospy.Publisher('odomPub', Odometry, queue_size=10)

    def sub_position_handler(self, position_info):
        self.x, self.y, _ = position_info

    def sub_attitude_info_handler(self, attitude_info):
        self.yaw, _, _ = attitude_info



    def publish_odometry(self):

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        rate=rospy.Rate(10)

        while (1):
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = -1*self.y
            odom.pose.pose.position.z = 0.0
            q = get_quaternion_from_euler(0, 0, radians(self.yaw))
            odom.pose.pose.orientation.x=q[0]
            odom.pose.pose.orientation.y=q[1]
            odom.pose.pose.orientation.z=q[2]
            odom.pose.pose.orientation.w=q[3]
            self.odom_pub.publish(odom)
            rate.sleep()


if __name__ == '__main__':
    robo = odom_publisher()
    time.sleep(0.5)
    robo.publish_odometry()
