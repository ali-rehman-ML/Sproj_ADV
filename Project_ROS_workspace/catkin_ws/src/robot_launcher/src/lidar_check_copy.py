#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np

#np.set_printoptions(precision=3)

lidar_brake_check=None

def callback(data):
	# -90 to -20 and 20 to 90 threshold 2
	# -20 to 20 3.5
	global lidar_brake_check
	scan_array = data.ranges

	l1=360-50
	l2=360-21 

	c1=360-20
	c2= 20 

	r1=21
	r2=50 


	readings=len(scan_array)

	indexl1=int(l1*readings/360)
	indexl2=int(l2*readings/360)

	indexc1=int(c1*readings/360)
	indexc2=int(c2*readings/360)

	indexr1=int(r1*readings/360)
	indexr2=int(r2*readings/360)
	
	left=scan_array[indexl1:indexl2]
	right=scan_array[indexr1:indexr2]

	center1=scan_array[indexc1:]
	center2=scan_array[:indexc2]

	center =np.array(center1+center2)

	if min(right)<2.5 or min(left)<2.5 or min(center)<3.5:
		lidar_brake_check=True
	else:
		lidar_brake_check=False



def pub_sub():
	global lidar_brake_check
	rospy.init_node('lidar_check', anonymous=True)
	rospy.Subscriber("scan", LaserScan, callback)
	pub = rospy.Publisher('brake', Bool, queue_size=10)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		pub.publish(lidar_brake_check)
		rate.sleep()
	rospy.spin()

if __name__ == '__main__':
	    pub_sub()

