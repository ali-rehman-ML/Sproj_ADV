#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import sin, cos, pi, degrees
import time


rospy.init_node('wheel_odometory2')

class wheel_ticks:
    def __init__(self,topic):
        self.current_ticks=0
        self.prev_ticks=0
        self.topic=topic
        rospy.Subscriber(self.topic, Int32, self.update_count)

    def update_count(self,msg):
        new_ticks=msg.data
        self.current_ticks=new_ticks
    def get_delta_ticks(self):
        curr_ticks=self.current_ticks
        del_ticks=curr_ticks-self.prev_ticks
        self.prev_ticks=curr_ticks
        
        
        return del_ticks 

class Wheel_Odometory:
    def __init__(self):
        self.x=0
        self.y=0
        self.theta=0
        self.curr_time=rospy.get_time()
        self.prev_time=rospy.get_time()
        self.delta_time=0
        self.odomPub = rospy.Publisher('wheel_odom', Odometry, queue_size=10)
        self.odom=Odometry()

        self.left_wheel=wheel_ticks("/leftTicks")
        self.right_wheel=wheel_ticks("/rightTicks")
        self.odomFrameID='base_link'
        self.baseFrameID='wheel_odom'
        self.ticks_per_meter=10
        self.wheel_radius=0.18
        self.wheel_base=1.15
        self.x_velocity=0
        self.y_velocity=0
        self.angular_z=0
        self.prev_theta=0
        self.linear_velocity=0

        self.rate=rospy.Rate(5)

    def kinematics(self):
        left_ticks=float(self.left_wheel.get_delta_ticks())
        right_ticks=float(self.right_wheel.get_delta_ticks())
    



        self.curr_time=rospy.get_time()
        self.delta_time=self.curr_time-self.prev_time
        self.prev_time=self.curr_time
        # print(left_ticks/1,right_ticks/1)
        print("left ticks ",left_ticks)
        left_displacement=(left_ticks/self.ticks_per_meter)*(2*pi*self.wheel_radius)
        right_displacement=(right_ticks/self.ticks_per_meter)*(2*pi*self.wheel_radius)
        left_velocity = left_displacement/self.delta_time
        right_velocity = right_displacement/self.delta_time
        self.linear_velocity = (left_velocity+right_velocity)/2
#        angular_velocity=(left_velocity - right_velocity) / self.wheel_base
        
        diff_ticks=(self.left_wheel.current_ticks-self.right_wheel.current_ticks)
        print('current ticks ',diff_ticks)

        # print("left disp", left_displacement)
        deltaTravel = (right_displacement + left_displacement) / 2
        self.theta =   (-1*self.left_wheel.current_ticks+self.right_wheel.current_ticks)*((2*pi*self.wheel_radius)/(10*self.wheel_base))#((self.right_wheel.current_ticks- self.right_wheel.current_ticks)*(2*pi*self.wheel_radius)) / (10*self.wheel_base)#(right_displacement - left_displacement) / self.wheel_base
        
        if abs(self.theta)>2*pi:
            self.theta=self.theta%(2*pi)
        deltaTheta=self.theta-self.prev_theta
        self.prev_theta=self.theta
        
        deltaX=0
        deltaY=0

        # deltaX = left_displacement*cos(deltaTheta)
        # deltaY = left_displacement*sin(deltaTheta)

        if deltaTheta==0:
            deltaX = left_displacement*cos(self.theta)
            deltaY = left_displacement*sin(self.theta)
            print("delta ", deltaX,deltaY,left_displacement)
        else:
        

            radius = deltaTravel / deltaTheta

            # Find the instantaneous center of curvature (ICC).
            iccX = self.x - radius*sin(self.theta)
            iccY = self.y + radius*cos(self.theta)
            deltaX = cos(deltaTheta)*(self.x - iccX) - sin(deltaTheta)*(self.y - iccY) + iccX - self.x
            deltaY = sin(deltaTheta)*(self.x - iccX) + cos(deltaTheta)*(self.y - iccY) + iccY - self.y
            # print("here ", deltaX)

        self.x += deltaX
        self.y += deltaY

        self.x_velocity=deltaX/self.delta_time if self.delta_time > 0 else 0
        self.y_velocity=0#deltaY/self.delta_time if self.delta_time > 0 else 0
        self.angular_z=deltaTheta/self.delta_time if self.delta_time > 0 else 0
        print(self.x,self.y,degrees(self.theta))


    def publish(self):
        while not rospy.is_shutdown():
            self.kinematics()
            odom = Odometry()
            q = quaternion_from_euler(0, 0, self.theta)
            odom.header.stamp = rospy.get_rostime()
            odom.header.frame_id = self.odomFrameID
            odom.child_frame_id = self.baseFrameID
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]
            odom.twist.twist.linear.x = self.linear_velocity
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = self.angular_z

            odom.pose.covariance = [10, 0, 0, 0, 0, 0,
                            0, 10, 0, 0, 0, 0,
                            0, 0, 10, 0, 0, 0,
                            0, 0, 0, 10, 0, 0,
                            0, 0, 0, 0, 10, 0,
                            0, 0, 0, 0, 0, 10]

            odom.twist.covariance=[10, 0, 0, 0, 0, 0,
                            0, 10, 0, 0, 0, 0,
                            0, 0, 10, 0, 0, 0,
                            0, 0, 0, 10, 0, 0,
                            0, 0, 0, 0, 10, 0,
                            0, 0, 0, 0, 0, 10]


            self.odomPub.publish(odom)
            self.rate.sleep()

if __name__ == '__main__':
    odom=Wheel_Odometory()
    time.sleep(0.5)
    odom.publish()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
##!/usr/bin/env python3

#import rospy
#from nav_msgs.msg import Odometry
#from std_msgs.msg import Int32
#from tf.transformations import quaternion_from_euler, euler_from_quaternion
#from math import sin, cos, pi, degrees
#import time

#rospy.init_node('wheel_odom')
#class state_width:
#    def __init__(self,topic):
#        self.topic=topic
#        self.widthsub=0
#        rospy.Subscriber(self.topic, Int32, self.widthCB)
#    def widthCB(self,msg):
#        self.widthsub=msg.data / 1000000
#    def width(self):
#        return self.widthsub
#    
#        
#class Wheel_Odometory:
#    def __init__(self):
#        self.position_x=0
#        self.position_y=0
#        self.theta=0
#        self.left_ticks=0
#        self.right_ticks=0
#        self.odomPub = rospy.Publisher('wheel_odom', Odometry, queue_size=10)
#        self.odom=Odometry() 
#        self.left_low_width=state_width("/left_low")
#        self.right_low_width=state_width("/right_low")
#        self.left_high_width=state_width("/left_high")
#        self.right_high_width=state_width("/right_high")
#        self.left_low_prev=0
#        self.left_high_prev=0
#        self.right_low_prev=0
#        self.right_high_prev=0
#        self.odomFrameID='odom'
#        self.baseFrameID='base_link'
#        self.ticks_per_rev=10
#        self.wheel_radius=0.18
#        self.wheel_base=1.15
#        self.linear_velocity=0
#        self.angular_velocity=0
#        self.x_velocity=0
#        self.y_velocity=0
#        self.angular_z=0
##        self.rate=rospy.Rate(50)
#        
#    def kinematics(self):
#        left_low=self.left_low_width.width()
#        right_low=self.right_low_width.width()
#        left_high=self.left_high_width.width()
#        right_high=self.right_high_width.width()
#        circumference=self.wheel_radius*2*pi
#        dist_per_tick=circumference/self.ticks_per_rev ## Distance travelled in one tick

#        if left_low<0.005 and (0.012>left_high>0.005): left_low=(left_high+self.left_low_prev)/2
#        if left_high<0.005 and (0.012>left_low>0.005): left_high=(left_low+self.left_high_prev)/2
#        if right_low<0.005 and (0.012>right_high>0.005): right_low=(right_high+self.right_low_prev)/2
#        if right_high<0.005 and (0.012>right_high>0.005): right_high=(right_low+self.right_high_prev)/2
#        left_time=left_low+left_high
#        right_time=right_low+right_high
#        if left_time != 0: self.left_ticks += 1
#        if right_time != 0: self.right_ticks += 1
#        
#        dist_per_tick=dist_per_tick=dist_per_tick
#        dt=(right_time+left_time)/2
#        # Calculate linear and angular velocities
#        if left_time>0 and right_time>0:
#            left_vel=dist_per_tick/(left_time)
#            right_vel=dist_per_tick/(right_time)
#            left_linear_velocity=dist_per_tick / left_time
#            right_linear_velocity=dist_per_tick / right_time
#            self.linear_velocity=(left_linear_velocity + right_linear_velocity) / 2.0
#            self.angular_velocity=(left_linear_velocity - right_linear_velocity) / self.wheel_base
##            delta_s = (dist_per_tick + dist_per_tick) / 2
#            delta_s=self.linear_velocity*dt
#            delta_theta = self.angular_velocity * dt
#        else: left_linear_velocity=right_linear_velocity=self.linear_velocity=self.angular_velocity=delta_s=delta_theta = 0
#        # Calculate position
#        self.position_x += delta_s * cos(self.theta + delta_theta / 2)
#        self.position_y += delta_s * sin(self.theta + delta_theta / 2)
#        self.theta += delta_theta
#        print(self.left_ticks, self.right_ticks)
##        print(left_time, right_time, self.position_x, self.position_y, self.linear_velocity, self.angular_velocity)
#        
#    def publish(self):
#        while not rospy.is_shutdown():
#            self.kinematics()
#            odom = Odometry()
#            q = quaternion_from_euler(0, 0, self.theta)
#            odom.header.stamp = rospy.get_rostime()
#            odom.header.frame_id = self.odomFrameID
#            odom.child_frame_id = self.baseFrameID
#            odom.pose.pose.position.x = self.position_x
#            odom.pose.pose.position.y = self.position_y
#            odom.pose.pose.orientation.x = q[0]
#            odom.pose.pose.orientation.y = q[1]
#            odom.pose.pose.orientation.z = q[2]
#            odom.pose.pose.orientation.w = q[3]
#            odom.twist.twist.linear.x = self.linear_velocity
#            odom.twist.twist.linear.y= 0 #self.y_velocity
#            odom.twist.twist.angular.z = self.angular_velocity
#            odom.pose.covariance = [0.03, 0, 0, 0, 0, 0,
#                                            0, 0.03, 0, 0, 0, 0,
#                                            0, 0, 0.03, 0, 0, 0,
#                                            0, 0, 0, 0.03, 0, 0,
#                                            0, 0, 0, 0, 0.03, 0,
#                                            0, 0, 0, 0, 0, 0.03]
#            self.odomPub.publish(odom)
##            self.rate.sleep()

#if __name__ == '__main__':
#    odom=Wheel_Odometory()
###    time.sleep(0.5)
#    odom.publish()


