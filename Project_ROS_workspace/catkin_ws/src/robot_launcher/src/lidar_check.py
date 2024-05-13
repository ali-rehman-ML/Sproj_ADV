#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np

#np.set_printoptions(precision=3)

lidar_brake_check=None

def callback(data):
	global lidar_brake_check
	scan_array = data.ranges
	p1=-50 #right
	rate = rospy.Rate(10)
	p2=50 #left

	readings=len(scan_array)

	index1=int(p1*readings/360)
	index2=int(p2*readings/360)
	l1=scan_array[index1:]
	l2=scan_array[:index2]
	a=np.array(l1+l2)
	if np.any(a<3.5):
		lidar_brake_check=True
	else:
		lidar_brake_check=False



def pub_sub():
	global lidar_brake_check
	rospy.init_node('lidar_check', anonymous=True)
	rospy.Subscriber("scan", LaserScan, callback)
	pub = rospy.Publisher('obstcale_brake', Bool, queue_size=10)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		pub.publish(lidar_brake_check)
		rate.sleep()
	rospy.spin()

if __name__ == '__main__':
	    pub_sub()

