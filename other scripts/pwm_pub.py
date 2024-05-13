#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import Int32, Bool


rospy.init_node('pwm_publisher')

count = 0
rate=rospy.Rate(10)

left_pub=rospy.Publisher("/pwm_left",Int32,queue_size=10)
right_pub=rospy.Publisher("/pwm_right",Int32,queue_size=10)
left_brake = rospy.Publisher('/left_brake', Bool, queue_size=10)
right_brake = rospy.Publisher('/right_brake', Bool, queue_size=10)
brake_both = rospy.Publisher('/brake', Bool, queue_size=10)


# while (1):
for i in range(50):
    left_msg=Int32()
    right_msg=Int32()
    left_msg.data=100
    right_msg.data=100

    left_pub.publish(left_msg)
    right_pub.publish(right_msg)
    left_brake.publish(False)
    right_brake.publish(False)
    brake_both.publish(False)
    rate.sleep()
    count += 1
for i in range(50):
    left_msg=Int32()
    right_msg=Int32()
    left_msg.data=0
    right_msg.data=100

    left_pub.publish(left_msg)
    right_pub.publish(right_msg)
    # right_brake.publish(False)
    # brake_both.publish(False)
    left_brake.publish(True)


    rate.sleep()
    print("left brake")
for i in range(50):
    left_msg=Int32()
    right_msg=Int32()
    left_msg.data=100
    right_msg.data=0

    left_pub.publish(left_msg)
    right_pub.publish(right_msg)
    # left_brake.publish(False)
    right_brake.publish(True)
    rate.sleep()
    print("right brake")




    
    
    
    
    
    


