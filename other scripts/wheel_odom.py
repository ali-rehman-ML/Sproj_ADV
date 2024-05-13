#! /usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import sin, cos, pi, degrees
import time
from std_msgs.msg import Float32

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
        self.odomPub = rospy.Publisher('odom_ticks', Odometry, queue_size=10)
        self.odom=Odometry()
        rospy.Subscriber("/imu_heading",Float32,self.update_heading)
        self.left_wheel=wheel_ticks("/left")
        self.right_wheel=wheel_ticks("/right")
        self.odomFrameID='odom_ticks'
        self.baseFrameID='base_link'
        self.ticks_per_meter=10
        self.wheel_radius=0.18
        self.wheel_base=1.15
        self.x_velocity=0
        self.y_velocity=0
        self.angular_z=0
        self.prev_theta=0

        self.rate=rospy.Rate(40)
    def update_heading(self,msg):
        self.theta=msg.data
    def kinematics(self):
        left_ticks=self.left_wheel.get_delta_ticks()
        right_ticks=self.right_wheel.get_delta_ticks()
    

        print(left_ticks,right_ticks)

        left_displacemnt=(left_ticks/self.ticks_per_meter)*(2*pi*self.wheel_radius)
        right_displacemnt=(right_ticks/self.ticks_per_meter)*(2*pi*self.wheel_radius)
        self.curr_time=rospy.get_time()
        self.delta_time=self.curr_time-self.prev_time
        self.prev_time=self.curr_time
        diff_ticks=(self.left_wheel.current_ticks-self.right_wheel.current_ticks)
        print('current ticks ',diff_ticks)


        deltaTravel = (right_displacemnt + left_displacemnt) / 2
#        self.theta =   (self.left_wheel.current_ticks-self.right_wheel.current_ticks)*((2*pi*self.wheel_radius)/(10*self.wheel_base))#((self.right_wheel.current_ticks- self.right_wheel.current_ticks)*(2*pi*self.wheel_radius)) / (10*self.wheel_base)#(right_displacemnt - left_displacemnt) / self.wheel_base
        
        if abs(self.theta)>2*pi:
            self.theta=self.theta%(2*pi)

        deltaTheta=self.theta-self.prev_theta
        self.prev_theta=self.theta
        
        deltaX=0
        deltaY=0
        if deltaTheta==0:
            deltaX = left_displacemnt*cos(self.theta)
            deltaY = left_displacemnt*sin(self.theta)
        else:
        

            radius = deltaTravel / deltaTheta

            # Find the instantaneous center of curvature (ICC).
            iccX = self.x - radius*sin(self.theta)
            iccY = self.y + radius*cos(self.theta)
            deltaX = cos(deltaTheta)*(self.x - iccX) - sin(deltaTheta)*(self.y - iccY) + iccX - self.x
            deltaY = sin(deltaTheta)*(self.x - iccX) + cos(deltaTheta)*(self.y - iccY) + iccY - self.y
            
        print("init ",self.x,self.y,deltaX,deltaY)
        self.x += deltaX
        self.y += deltaY

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



        


        

    

        



































###!/usr/bin/env python3

##import time
##import Jetson.GPIO as GPIO
##import rospy
##from geometry_msgs.msg import Twist
##from nav_msgs.msg import Odometry
##from std_msgs.msg import Float64, Int32
##from tf.transformations import quaternion_from_euler
##from math import pi, cos, sin


###left_hall_sensor_pin = 31
###right_hall_sensor_pin = 33
##pulses_per_revolution = 10  # Number of pulses generated per revolution
##wheel_radius = 0.18  # Wheel radius in meters
##wheel_base = 1.15  # Wheel base in meters
##left_start_time = []
##right_start_time = []
##left_time = 0.0
##right_time = 0.0
##left_pulse_count = 0.0 
##right_pulse_count = 0.0
##moving_average_window = 2
##  # Number of RPM values to use for moving average
##left_rpm_values = []  # List to store last RPM values for left wheel
##right_rpm_values = []  # List to store last RPM values for right wheel
##linear_velocity = 0.0  # Linear velocity in m/s
##angular_velocity = 0.0  # Angular velocity in rad/s
##position_x = 0.0  # X position in meters
##position_y = 0.0  # Y position in meters
##theta = 0.0  # Orientation in radians


##def calculate_moving_average(rpm_values):
##    if len(rpm_values) > 0:  return rpm_values[-1] - rpm_values[0] 
##    else: return 0.0

##def leftCallback(data):
##    global left_start_time, left_pulse_count, left_time, left_rpm_values
##    current_time = rospy.get_time() 
##    left_start_time.append(current_time)
##    left_rpm_values.append(data.data)
##    
##    if len(left_start_time) == 0 or len(left_start_time) == 1: 
##        left_pulse_count += 1
##        left_time = 0.05 ## Initialized it to 1sec
##    else:
##        left_time = left_start_time[-1] - left_start_time[0]
###        left_time = left_time * 2
##        if len(left_rpm_values) > moving_average_window: left_rpm_values.pop(0)
##        if len(left_start_time) > moving_average_window: left_start_time.pop(0)


##def rightCallback(data):
##    global right_start_time, right_pulse_count, right_time, right_rpm_values
##    current_time = rospy.get_time() 
##    right_start_time.append(current_time)
##    right_rpm_values.append(data.data)
##    
##    if len(right_start_time) == 0 or len(right_start_time) == 1: 
##        right_pulse_count += 1
##        right_time = 0.05 ## Initialized it to 1sec
##    else:
##        right_time = right_start_time[-1] - right_start_time[0]
###        right_time = right_time * 2
##        if len(right_rpm_values) > moving_average_window: right_rpm_values.pop(0)
##        if len(right_start_time) > moving_average_window: right_start_time.pop(0)
##    

##def calculate_odometry():
##    global left_rpm_values, right_rpm_values, position_x, position_y, theta

##    # Check if both RPM values are available
##    if len(left_rpm_values) > 0 and  len(right_rpm_values) > 0:
##        moving_average_left = calculate_moving_average(left_rpm_values)
##        moving_average_right = calculate_moving_average(right_rpm_values)
###        rospy.loginfo("Left Wheel RPM (Moving Average): %.2f", moving_average_left)
###        rospy.loginfo("Right Wheel RPM (Moving Average): %.2f", moving_average_right)
##        left_rpm_pub.publish(moving_average_left)
##        right_rpm_pub.publish(moving_average_right)
##	
##        distance_left = (2 * pi * wheel_radius * moving_average_left) / 10
##        distance_right = (2 * pi * wheel_radius * moving_average_right) / 10
##        dt = (right_time + left_time) / 2
##        # Calculate linear and angular velocities
##        left_linear_velocity = distance_left / left_time
##        right_linear_velocity = distance_right / right_time
##        linear_velocity = (left_linear_velocity + right_linear_velocity) / 2.0
##        angular_velocity = (left_linear_velocity - right_linear_velocity) / wheel_base
##        delta_s = (distance_left + distance_right) / 2
##        delta_theta = angular_velocity * dt
##        print(left_linear_velocity, right_linear_velocity, linear_velocity, angular_velocity)

##        # Calculate position
##        print(dt, delta_theta, delta_s)
##        position_x += delta_s * cos(theta + delta_theta / 2)
##        position_y += delta_s * sin(theta + delta_theta / 2)
##        theta += delta_theta


######    //since all odometry is 6DOF we'll need a quaternion created from yaw
######    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

######    //first, we'll publish the transform over tf
######    geometry_msgs::TransformStamped odom_trans;
######    odom_trans.header.stamp = current_time;
######    odom_trans.header.frame_id = "odom";
######    odom_trans.child_frame_id = "base_link";

######    odom_trans.transform.translation.x = x;
######    odom_trans.transform.translation.y = y;
######    odom_trans.transform.translation.z = 0.0;
######    odom_trans.transform.rotation = odom_quat;

######    //send the transform
######    odom_broadcaster.sendTransform(odom_trans);
##    
##        # Publish Odometry message
##        quaternion = quaternion_from_euler(0, 0, theta)
##        odom = Odometry()
##        odom.header.stamp = rospy.Time.now()
##        odom.header.frame_id = "odom"
##        odom.child_frame_id = "base_link"
##        odom.pose.pose.position.x = position_x
##        odom.pose.pose.position.y = position_y
##        odom.pose.pose.position.z = 0.0;
###        odom.pose.pose.orientation = quaternion
##        odom.twist.twist.linear.x = linear_velocity
##        odom.twist.twist.angular.z = angular_velocity
##        odom.pose.pose.orientation.x = quaternion[0]
##        odom.pose.pose.orientation.y = quaternion[1]
##        odom.pose.pose.orientation.z = quaternion[2]
##        odom.pose.pose.orientation.w = quaternion[3]


##        # Set covariance matrix with diagonal values 0.03
##        odom.pose.covariance = [0.03, 0, 0, 0, 0, 0,
##                                0, 0.03, 0, 0, 0, 0,
##                                0, 0, 0.03, 0, 0, 0,
##                                0, 0, 0, 0.03, 0, 0,
##                                0, 0, 0, 0, 0.03, 0,
##                                0, 0, 0, 0, 0, 0.03]

##        odom_pub.publish(odom)

##        # Clear the RPM lists
###        if len(left_rpm_values) >= moving_average_window: left_rpm_values = []
###        if len(right_rpm_values) >= moving_average_window: right_rpm_values = []

##if __name__ == '__main__':
##    rospy.init_node('odometry_publisher')
##    rospy.Subscriber('left', Int32, leftCallback)
##    rospy.Subscriber('right', Int32, rightCallback)
##    
##    odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
##    left_rpm_pub = rospy.Publisher('left_rpm', Float64, queue_size=10)
##    right_rpm_pub = rospy.Publisher('right_rpm', Float64, queue_size=10)
##    
##    try:
##        while not rospy.is_shutdown():
##            calculate_odometry()
##            time.sleep(0.1)
##    except rospy.ROSInterruptException:
##        pass

