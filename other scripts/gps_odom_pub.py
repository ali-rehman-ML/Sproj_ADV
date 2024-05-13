#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from math import degrees, radians, sin,cos, atan2
import  time
import math
from math import sin, cos, pi
import pynmea2
import serial
import string
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import numpy as np #

def convert(Lat, Lon): 
    Lat = str(Lat)
    Latdd = float(Lat[:2])
    Latmmmmm = float(Lat[2:])
    Latddmmmm = Latdd+(Latmmmmm/60.0)
    Lon = str(Lon)
    Londd = float(Lon[:3])
    Lonmmmmm = float(Lon[3:])
    Londdmmmm = Londd+(Lonmmmmm/60.0)
    return Latddmmmm, Londdmmmm


 
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
class gps_odom_pub:
    def __init__(self):
        self.x=0
        self.y=0
        self.theta=0
        self.initial_heading=None
        self.Current_heading=None
        self.initi_lat=0
        self.initi_long=0
        self.curr_lat=0
        self.curr_long=0
        rospy.init_node('gps_odom', anonymous=True)

        rospy.Subscriber("/compass_heading", Int32, self.heading_call_back)
        self.odom_pub = rospy.Publisher("gps", Odometry, queue_size=50)
        self.curre_time=rospy.get_time()
        self.prev_time=rospy.get_time()


    def heading_call_back(self,msg):
        h=msg.data
        
        if self.initial_heading is None:
            self.initial_heading=h
            self.Current_heading=h
            
        else :
            self.Current_heading=h
            
        self.theta=self.Current_heading-self.initial_heading

    def transform(self):

        if 31.0<self.curr_lat<32.0 and 70.0 <self.curr_long<76 and self.initi_lat>0 and self.initi_long>0:

        
            tx=(self.curr_lat-self.initi_lat)*(111131)
            ty=(self.curr_long-self.initi_long)*(111131)
        
            theta=radians(self.Current_heading)
        
            self.x=cos(theta)*tx +sin(theta)*ty
            self.y=-sin(theta)*tx + cos(theta)*ty

            return self.x,self.y,radians(self.theta)

        else:
            return self.x,self.y,radians(self.theta)


    def publish_data(self):
        true_lat = 0.0
        true_lon = 0.0
        count = 0
        last_time = rospy.Time.now()
        r = rospy.Rate(0.5)

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            ser = serial.Serial("/dev/ttyTHS1", baudrate=9600, timeout=1)
            while True:
                try:
                    data = ser.readline()
                    data = data.decode("utf-8").strip()
                    break
                except UnicodeDecodeError:
                    continue

            check = False
            lat = 0
            lon = 0
            alt = 0

            while ser.inWaiting() > 0: 
                while True:
                    try:
                        data = ser.readline()
                        data = data.decode("utf-8").strip()
                        break
                    except UnicodeDecodeError:
                        continue
                if data[1:6]=="GPGGA": 
                    check = True
                    newmsg=pynmea2.parse(data)
                    if newmsg.lat != "":
                        lat, lon = convert(newmsg.lat, newmsg.lon)
                        alt = newmsg.altitude
                        alt = float(alt)
                        time = newmsg.timestamp 

                    if lat=="": check = False


            if self.initi_lat==0 and 31.0 <float(lat)< 32 and 72.0<float(lon)<76:
                self.initi_lat=float(lat)
                self.initi_long=float(lon)
            self.curr_lat=float(lat)
            self.curr_long=float(lon)

            x,y,theta=self.transform()
            quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)

            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "gps"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = x                        # x measurement GPS.
            odom.pose.pose.position.y = y                         # y measurement GPS.
            odom.pose.pose.position.z = 0                           # z measurement GPS.
            odom.pose.pose.orientation.x = quaternion[0]                        # identity quaternion
            odom.pose.pose.orientation.y = quaternion[1]                        # identity quaternion
            odom.pose.pose.orientation.z = quaternion[2]                        # identity quaternion
            odom.pose.pose.orientation.w = quaternion[3]                       # identity quaternion
            odom.pose.covariance = [0.03, 0, 0, 0, 0, 0,  0, 0.03, 0, 0, 0, 0,  0, 0, 0.03, 0, 0, 0,  0, 0, 0, 99999, 0, 0,  0, 0, 0, 0, 99999, 0,  0, 0, 0, 0, 0, 99999]
            odom_pub.publish(odom)
            last_time = current_time
            r.sleep()



if __name__ == '__main__':
    odom=gps_odom_pub()
    time.sleep(1)
    odom.publish_data()