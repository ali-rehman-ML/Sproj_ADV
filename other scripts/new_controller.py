#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
from math import pow, atan2, sqrt, radians,degrees
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Int32,Bool
from sensor_msgs.msg import Imu



class Goto_Goal:
    def __init__(self):
        self.kp_turn=2
        self.ki_turn=0.1
        self.kd_turn=-0.1
        self.x=0
        self.y=0
        self.theta=0
        self.current_e_turning=0
        self.previous_e_turning=0
        self.E_int_turning=0
        self.r=0.18
        self.b=1.15
        self.dt=0.1



        #heading controller variable
        self.error=0
        self.initial_heading=-1000
        self.current_heading=0
        self.kp_heading=15
        self.ki_heading=0.1
        self.kd_heading=-0.15
        self.current_e_heading=0
        self.previous_e_heading=0
        self.E_int_heading=0

        # Ros topics
        rospy.init_node('goto_goal_controller')
        rospy.Subscriber("/odom_ticks", Odometry, self.call_back)
        rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.left_pub=rospy.Publisher("/pwm_left",Int32,queue_size=10)
        self.right_pub=rospy.Publisher("/pwm_right",Int32,queue_size=10)
        self.pub = rospy.Publisher('/controller_brake', Bool, queue_size=10)
        self.rate=rospy.Rate(10)

        #goal parameters
        self.goal_x=3
        self.goal_y=1
        self.tolerance=1


    def imu_callback(self,data):
        _,_,yaw_rad=euler_from_quaternion([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w])
        
        self.current_heading=math.degrees(yaw_rad)
        # print("callback ", self.current_heading)
        if self.initial_heading == -1000:
            print(self.initial_heading)
            self.initial_heading=self.current_heading

    def call_back(self,data):
        self.x=round(data.pose.pose.position.x,4)
        self.y=round(data.pose.pose.position.y,4)
        
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion (orientation_list)
        
        yaw=math.degrees(yaw)

        self.theta=yaw

    def PID_heading(self,e):
        self.current_e_heading=e

        e_d=(self.current_e_heading-self.previous_e_heading)/self.dt
        e_i=self.E_int_heading+self.current_e_heading*self.dt
        self.previous_e_heading=self.current_e_heading

        E_pid=self.kp_heading*self.current_e_heading + self.ki_heading*e_i + self.kd_heading*e_d
        return E_pid
    

    def PID_turning(self,e):
        self.current_e_turning=e

        e_d=(self.current_e_turning-self.previous_e_turning)/self.dt
        e_i=self.E_int_turning+self.current_e_turning*self.dt
        self.previous_e_turning=self.current_e_turning

        E_pid=self.kp_turn*self.current_e_turning + self.ki_turn*e_i + self.kd_turn*e_d
        return E_pid

    def goto_goal_straight(self):
        current_location=np.array([self.x,self.y])
        goal_location=np.array([self.goal_x,self.goal_y])
        distance=np.linalg.norm(current_location-goal_location)
        while(distance>self.tolerance):
            distance=np.linalg.norm(current_location-goal_location)
            pwmmsg_left=Int32()
            pwmmsg_right=Int32()

            self.error=self.initial_heading-self.current_heading

            if abs(self.error)<0.2:
                self.error=0

	        if abs(self.error)>50:

	            if self.error>0:

	                self.error-=360
	            else:
	                self.error+=360


            e_pid=self.PID_heading(self.error)

            if abs(self.error)>5:
                e_pid=np.clip(e_pid,-40,40)
            else:
                e_pid=np.clip(e_pid,-20,20)
            
            pwmmsg_right.data=int(95+e_pid)
            pwmmsg_left.data=int(95-e_pid)   
            self.left_pub.publish(pwmmsg_left)
            self.right_pub.publish(pwmmsg_right)
            self.rate.sleep()

    def goto_goal_turning(self):
            r=1#self.r
            b=self.b
            a=1/(2*b)
            # R=np.array([[r/2,r/2],
            #            [r/(2*b),(-1*r)/(2*b)]])
            R=np.array([[1/2,1/2],
                        [10/23,(-10/23)]])
            inv_R=np.array([[1,23/20],
                            [1,-23/20]])
            current_location=np.array([self.x,self.y])
            goal_location=np.array([self.goal_x,self.goal_y])

            distance=np.linalg.norm(current_location-goal_location)


            while(distance>self.tolerance):
                distance=np.linalg.norm(current_location-goal_location)
                des_angle=degrees(atan2((self.goal_y-self.y),(self.goal_x-self.x)))



                angle_error=des_angle-self.theta

                angular_velocity=self.PID_turning(angle_error)


               # angular_velocity=np.clip(angular_velocity,-90,90)
                if abs(angle_error)>40:
                    lin_vel=distance/abs(angular_velocity)
                else:
        
                    lin_vel= distance#distance*0.1/(angular_velocity) if abs(angle_error)>10 else distance*10
                    
#                lin_vel= distance*0.1/(angular_velocity) if abs(angle_error)>10 else distance*10
                angular_velocity=radians(angular_velocity)

                lin_vel=np.clip(lin_vel,-1.5,1.5)

                velocities=np.array([lin_vel,(angular_velocity)]).reshape(2,1)

                rpm=np.dot(inv_R,velocities).flatten()
                
                
                print("angular vel ", velocities)
                print("rpm : ",rpm)
                ticks_per_sec=np.clip(rpm*(10),0,30)
                
                # print("distance ", distance,current_location)
                pwm=ticks_per_sec*2 + 90
                
                print("ticker per sec ", ticks_per_sec,pwm)
                pwm=np.clip(pwm,0,120)                
                current_location=np.array([self.x,self.y])
                distance=np.linalg.norm(current_location-goal_location)
                left_msg=Int32()
                right_msg=Int32()
                left_msg.data=int(pwm[1])
                
                right_msg.data=int(pwm[0])
            
                self.left_pub.publish(left_msg)
                self.right_pub.publish(right_msg)
                self.pub.publish(False)
                self.rate.sleep()

        


    def goto_goal(self):
            for _ in range(60):
                left_msg=Int32()
                right_msg=Int32()
                left_msg.data=50
                right_msg.data=50
                    
                self.left_pub.publish(left_msg)
                self.right_pub.publish(right_msg)
                self.rate.sleep()


            bearing=degrees(atan2((self.goal_y-self.y),(self.goal_x-self.x)))

            if abs(bearing)<10:
                print("heading controll")
                self.goto_goal_straight()
                
            else:
                print("turning controller")
                self.goto_goal_turning()

            while(1):
                print("zero pub goal reached")
                left_msg=Int32()
                right_msg=Int32()
                left_msg.data=0
                right_msg.data=0
                self.pub.publish(True)    
                self.left_pub.publish(left_msg)
                self.right_pub.publish(right_msg)
                self.rate.sleep()


if __name__ == '__main__':
    robo=Goto_Goal()
    time.sleep(1)
    robo.goto_goal()
                            


                


            


