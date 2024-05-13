#!/usr/bin/env python2

# Copyright (c) 2012, Tang Tiong Yew
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# Changed

import rospy
import serial
import string
import math
import sys

#from time import time

from std_msgs.msg import Float32
from razor_imu_9dof.cfg import imuConfig

degrees2rad = math.pi/180.0
imu_yaw_calibration = 0.0

# Callback for dynamic reconfigure requests

rospy.init_node("razor_node")


# read basic information
port = rospy.get_param('~port', '/dev/ttyUSB0')
topic = rospy.get_param('~topic', 'imu')
frame_id = rospy.get_param('~frame_id', 'base_imu_link')

# read calibration parameters

# accelerometer
accel_x_min = rospy.get_param('~accel_x_min', -250.0)
accel_x_max = rospy.get_param('~accel_x_max', 250.0)
accel_y_min = rospy.get_param('~accel_y_min', -250.0)
accel_y_max = rospy.get_param('~accel_y_max', 250.0)
accel_z_min = rospy.get_param('~accel_z_min', -250.0)
accel_z_max = rospy.get_param('~accel_z_max', 250.0)

# magnetometer
magn_x_min = rospy.get_param('~magn_x_min', -600.0)
magn_x_max = rospy.get_param('~magn_x_max', 600.0)
magn_y_min = rospy.get_param('~magn_y_min', -600.0)
magn_y_max = rospy.get_param('~magn_y_max', 600.0)
magn_z_min = rospy.get_param('~magn_z_min', -600.0)
magn_z_max = rospy.get_param('~magn_z_max', 600.0)
calibration_magn_use_extended = rospy.get_param('~calibration_magn_use_extended', False)
magn_ellipsoid_center = rospy.get_param('~magn_ellipsoid_center', [0, 0, 0])
magn_ellipsoid_transform = rospy.get_param('~magn_ellipsoid_transform', [[0, 0, 0], [0, 0, 0], [0, 0, 0]])
imu_yaw_calibration = rospy.get_param('~imu_yaw_calibration', 0.0)

# gyroscope
gyro_average_offset_x = rospy.get_param('~gyro_average_offset_x', 0.0)
gyro_average_offset_y = rospy.get_param('~gyro_average_offset_y', 0.0)
gyro_average_offset_z = rospy.get_param('~gyro_average_offset_z', 0.0)

yaw_pub = rospy.Publisher('yaw', Float32 , queue_size=10)
yaw_pub_time = rospy.get_time();

# Check your COM port and baud rate
rospy.loginfo("Opening %s...", port)
try:
    ser = serial.Serial(port=port, baudrate=57600, timeout=1)
    #ser = serial.Serial(port=port, baudrate=57600, timeout=1, rtscts=True, dsrdtr=True) # For compatibility with some virtual serial ports (e.g. created by socat) in Python 2.7
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port "+port + ". Did you specify the correct port in the launch file?")
    #exit
    sys.exit(2)

roll=0
pitch=0
yaw=0
seq=0
accel_factor = 9.806 / 256.0    # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.
rospy.loginfo("Giving the razor IMU board 5 seconds to boot...")
rospy.sleep(5) # Sleep for 5 seconds to wait for the board to boot

### configure board ###
#stop datastream
ser.write(('#o0').encode("utf-8"))

#discard old input
#automatic flush - NOT WORKING
#ser.flushInput()  #discard old input, still in invalid format
#flush manually, as above command is not working
discard = ser.readlines() 

#set output mode
ser.write(('#ox').encode("utf-8")) # To start display angle and sensor reading in text

rospy.loginfo("Writing calibration values to razor IMU board...")
#set calibration values
ser.write(('#caxm' + str(accel_x_min)).encode("utf-8"))
ser.write(('#caxM' + str(accel_x_max)).encode("utf-8"))
ser.write(('#caym' + str(accel_y_min)).encode("utf-8"))
ser.write(('#cayM' + str(accel_y_max)).encode("utf-8"))
ser.write(('#cazm' + str(accel_z_min)).encode("utf-8"))
ser.write(('#cazM' + str(accel_z_max)).encode("utf-8"))

if (not calibration_magn_use_extended):
    ser.write(('#cmxm' + str(magn_x_min)).encode("utf-8"))
    ser.write(('#cmxM' + str(magn_x_max)).encode("utf-8"))
    ser.write(('#cmym' + str(magn_y_min)).encode("utf-8"))
    ser.write(('#cmyM' + str(magn_y_max)).encode("utf-8"))
    ser.write(('#cmzm' + str(magn_z_min)).encode("utf-8"))
    ser.write(('#cmzM' + str(magn_z_max)).encode("utf-8"))
else:
    ser.write(('#ccx' + str(magn_ellipsoid_center[0])).encode("utf-8"))
    ser.write(('#ccy' + str(magn_ellipsoid_center[1])).encode("utf-8"))
    ser.write(('#ccz' + str(magn_ellipsoid_center[2])).encode("utf-8"))
    ser.write(('#ctxX' + str(magn_ellipsoid_transform[0][0])).encode("utf-8"))
    ser.write(('#ctxY' + str(magn_ellipsoid_transform[0][1])).encode("utf-8"))
    ser.write(('#ctxZ' + str(magn_ellipsoid_transform[0][2])).encode("utf-8"))
    ser.write(('#ctyX' + str(magn_ellipsoid_transform[1][0])).encode("utf-8"))
    ser.write(('#ctyY' + str(magn_ellipsoid_transform[1][1])).encode("utf-8"))
    ser.write(('#ctyZ' + str(magn_ellipsoid_transform[1][2])).encode("utf-8"))
    ser.write(('#ctzX' + str(magn_ellipsoid_transform[2][0])).encode("utf-8"))
    ser.write(('#ctzY' + str(magn_ellipsoid_transform[2][1])).encode("utf-8"))
    ser.write(('#ctzZ' + str(magn_ellipsoid_transform[2][2])).encode("utf-8"))

ser.write(('#cgx' + str(gyro_average_offset_x)).encode("utf-8"))
ser.write(('#cgy' + str(gyro_average_offset_y)).encode("utf-8"))
ser.write(('#cgz' + str(gyro_average_offset_z)).encode("utf-8"))
ser.flushInput()
ser.write(('#p').encode("utf-8"))

calib_data = ser.readlines()
calib_data_print = "Printing set calibration values:\r\n"
for row in calib_data:
    line = bytearray(row).decode("utf-8")
    calib_data_print += line

#start datastream
ser.write(('#o1').encode("utf-8"))

#automatic flush - NOT WORKING
#ser.flushInput()  #discard old input, still in invalid format
#flush manually, as above command is not working - it breaks the serial connection
rospy.loginfo("Flushing first 200 IMU entries...")
for x in range(0, 200):
    line = bytearray(ser.readline()).decode("utf-8")
rospy.loginfo("Publishing IMU data...")

errcount = 0
while not rospy.is_shutdown():
    yaw=0
    if (errcount > 10):
        break
    line = bytearray(ser.readline()).decode("utf-8")

    if ((line.find("#YPRAG=") == -1) or (line.find("\r\n") == -1)): 
        rospy.logwarn("Bad IMU data or bad sync")
        errcount = errcount+1
        continue
    line = line.replace("#YPRAG=","")   # Delete "#YPRAG="
    line = line.replace("\r\n","")   # Delete "\r\n"
    words = line.split(",")    # Fields split
    if len(words) != 9:
        rospy.logwarn("Bad IMU data or bad sync")
        errcount = errcount+1
        continue
    else:
        errcount = 0
        yaw_deg = -float(words[0])
        yaw_deg = yaw_deg#imu_yaw_calibration
        #if yaw_deg > 180.0:
        #    yaw_deg = yaw_deg - 360.0
        #if yaw_deg < -180.0:
        #    yaw_deg = yaw_deg + 360.0
        #yaw = yaw_deg*degrees2rad


    if (yaw_pub_time < rospy.get_time()) :
        yaw_pub_time += 1
	yaw_pub.publish(yaw_deg)
        
ser.close

if (errcount > 10):
    sys.exit(10)