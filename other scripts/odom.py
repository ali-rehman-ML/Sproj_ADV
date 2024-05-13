#! /usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import sin, cos, pi, degrees
import time

rospy.init_node('wheel_odometory_2')
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

        self.left_wheel=wheel_ticks("/left")
        self.right_wheel=wheel_ticks("/right")
        self.odomFrameID='odom'
        self.baseFrameID='base_link'
        self.ticks_per_meter=10
        self.wheel_radius=0.18
        self.wheel_base=1.15
        self.x_velocity=0
        self.y_velocity=0
        self.angular_z=0

        self.rate=rospy.Rate(100)

    def kinematics(self):
        left_ticks=self.left_wheel.get_delta_ticks()
        right_ticks=self.right_wheel.get_delta_ticks()
#        print(left_ticks,right_ticks)

        left_displacemnt=(left_ticks/self.ticks_per_meter)*(2*pi*self.wheel_radius)
        right_displacemnt=(right_ticks/self.ticks_per_meter)*(2*pi*self.wheel_radius)
        self.curr_time=rospy.get_time()
        self.delta_time=self.curr_time-self.prev_time
        self.prev_time=self.curr_time


        deltaTravel = (right_displacemnt + left_displacemnt) / 2
        deltaTheta = ((self.right_wheel.current_ticks/10)*(2*pi*self.wheel_radius) - (self.right_wheel.current_ticks/10)*(2*pi*self.wheel_radius)) / self.wheel_base
        deltaX=0
        deltaY=0
        if right_displacemnt == left_displacemnt:
            deltaX = left_displacemnt*cos(self.theta)
            deltaY = left_displacemnt*sin(self.theta)
        else:
            radius = deltaTravel / deltaTheta if deltaTheta >0 else 0

            # Find the instantaneous center of curvature (ICC).
            iccX = self.x - radius*sin(self.theta)
            iccY = self.y + radius*cos(self.theta)

            deltaX = cos(deltaTheta)*(self.x - iccX) \
                - sin(deltaTheta)*(self.y - iccY) \
                + iccX - self.x

            deltaY = sin(deltaTheta)*(self.x - iccX) \
                + cos(deltaTheta)*(self.y - iccY) \
                + iccY - self.y
            
        self.x += deltaX
        self.y += deltaY
        self.theta=deltaTheta
        self.x_velocity=deltaX/self.delta_time if self.delta_time > 0 else 0
        self.y_velocity=deltaY/self.delta_time if self.delta_time > 0 else 0
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
            odom.twist.twist.linear.x = self.x_velocity
            odom.twist.twist.linear.y=self.y_velocity
            odom.twist.twist.angular.z = self.angular_z

            odom.pose.covariance = [0.03, 0, 0, 0, 0, 0,
                            0, 0.03, 0, 0, 0, 0,
                            0, 0, 0.03, 0, 0, 0,
                            0, 0, 0, 0.03, 0, 0,
                            0, 0, 0, 0, 0.03, 0,
                            0, 0, 0, 0, 0, 0.03]
            self.odomPub.publish(odom)
            self.rate.sleep()

if __name__ == '__main__':
    odom=Wheel_Odometory()
    time.sleep(0.5)
    odom.publish()



        


        

    

        












