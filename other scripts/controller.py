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

class Goto_Goal:
    def __init__(self):
        self.kp=2
        self.ki=0.1
        self.kd=-0.1
        self.x=0
        self.y=0
        self.theta=0
        self.current_e=0
        self.previous_e=0
        self.E_int=0
        self.r=0.18
        self.b=1.15
        self.goal_x=9
        self.goal_y=0
        self.tolerance=1
        self.dt=0.1
        rospy.init_node('goto_goal_controller')
        rospy.Subscriber("/rtabmap/visual_odom", Odometry, self.call_back)
        self.left_pub=rospy.Publisher("/pwm_left",Int32,queue_size=10)
        self.right_pub=rospy.Publisher("/pwm_right",Int32,queue_size=10)
        self.pub = rospy.Publisher('/controller_brake', Bool, queue_size=10)
        self.rate=rospy.Rate(10)


    def call_back(self,data):
            self.x=round(data.pose.pose.position.x,4)
            self.y=round(data.pose.pose.position.y,4)
            
            orientation_q = data.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            roll, pitch, yaw = euler_from_quaternion (orientation_list)
            
            yaw=math.degrees(yaw)

            self.theta=yaw

    def PID(self,e):
            self.current_e=e

            e_d=(self.current_e-self.previous_e)/self.dt
            e_i=self.E_int+self.current_e*self.dt
            self.previous_e=self.current_e

            E_pid=self.kp*self.current_e + self.ki*e_i + self.kd*e_d
            return E_pid
        


    def goto_goal(self):
            for _ in range(60):
                left_msg=Int32()
                right_msg=Int32()
                left_msg.data=50
                right_msg.data=50
                    
                self.left_pub.publish(left_msg)
                self.right_pub.publish(right_msg)
                self.rate.sleep()
            r=1#self.r
            b=self.b
            a=1/(2*b)
            # R=np.array([[r/2,r/2],
            #            [r/(2*b),(-1*r)/(2*b)]])
            R=np.array([[1/2,1/2],
                        [10/23,(-10/23)]])
            inv_R=np.array([[1,23/20],
                            [1,-23/20]])
            # print(np.linalg.inv(R))
            current_location=np.array([self.x,self.y])
            goal_location=np.array([self.goal_x,self.goal_y])

            distance=np.linalg.norm(current_location-goal_location)
            while(distance>self.tolerance):
                distance=np.linalg.norm(current_location-goal_location)
                des_angle=degrees(atan2((self.goal_y-self.y),(self.goal_x-self.x)))



                angle_error=des_angle-self.theta

                angular_velocity=self.PID(angle_error)


               # angular_velocity=np.clip(angular_velocity,-90,90)
                if abs(angle_error)>40:
                    lin_vel=distance/abs(angular_velocity)
                else:
        
                    lin_vel= distance#distance*0.1/(angular_velocity) if abs(angle_error)>10 else distance*10
                    
#                lin_vel= distance*0.1/(angular_velocity) if abs(angle_error)>10 else distance*10
                angular_velocity=radians(angular_velocity)

                lin_vel=np.clip(lin_vel,-1,1)

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
                            


                


            


