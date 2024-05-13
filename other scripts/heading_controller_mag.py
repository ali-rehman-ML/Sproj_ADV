#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32
from std_msgs.msg import Bool
import math
import time
import numpy as np
 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z 







class heading_controller:
    def __init__(self):
        self.error=0
        self.initial_heading=-1000
        self.current_heading=0
        self.kp=15
        self.ki=0.1
        self.kd=-0.15
        self.current_e=0
        self.previous_e=0
        self.E_int=0
        self.dt=0.1
        
        rospy.init_node('headig_controller')
        rospy.Subscriber("/compass_heading", Int32, self.heading_call_back)

        self.pwm_pub_left = rospy.Publisher('/pwm_left', Int32, queue_size=10)
        self.pwm_pub_right = rospy.Publisher('/pwm_right', Int32, queue_size=10)
        self.left_brake = rospy.Publisher('/left_brake', Bool, queue_size=10)
        self.right_brake = rospy.Publisher('/right_brake', Bool, queue_size=10)

        self.on_controller=True

            

        
    def heading_call_back(self,msg):
       
        
        
        self.current_heading=msg.data
        print("callback ", self.current_heading)
        if self.initial_heading == -1000:
            print(self.initial_heading)
            self.initial_heading=self.current_heading


    def PID(self,e):

        self.current_e=e

        e_d=(self.current_e-self.previous_e)/self.dt
        e_i=self.E_int+self.current_e*self.dt
        self.previous_e=self.current_e

        E_pid=self.kp*self.current_e + self.ki*e_i + self.kd*e_d
        return E_pid
        
        
    def pwm_publisher(self):
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            pwmmsg_left=Int32()
            pwmmsg_right=Int32()
            self.error=0
            if self.on_controller==True:
                self.error=self.initial_heading-self.current_heading
	        if abs(self.error)>50:

	            if self.error>0:

	                self.error-=360
	            else:
	                self.error+=360

            if abs(self.error)<=1:
                self.error=0

            

            e_pid=self.PID(self.error)
            e_pid=np.clip(e_pid,-20,20)
            
            pwmmsg_right.data=int(95+e_pid)
            pwmmsg_left.data=i h=msg.datant(95-e_pid)


                
                
                
            self.pwm_pub_left.publish(pwmmsg_left)
            self.pwm_pub_right.publish(pwmmsg_right)
            rate.sleep()
            print("error ",self.error)
	    print("heading ",self.current_heading)

if __name__ == '__main__':
    head_contr=heading_controller()
    time.sleep(3)
    head_contr.pwm_publisher()


