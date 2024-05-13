#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class IMUToYawConverter:
    def __init__(self):
        rospy.init_node('imu_to_yaw_converter', anonymous=True)

        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        

        self.imu_pub = rospy.Publisher('imu_compass', Imu, queue_size=10)
        self.heading_compass_pub = rospy.Publisher('headIng_north', Int32, queue_size=10)

        self.imu_msg=Imu()

    def imu_callback(self, msg):
        self.imu_msg=msg
        # Extract quaternion components
        q_x = msg.orientation.x
        q_y = msg.orientation.y
        q_z = msg.orientation.z
        q_w = msg.orientation.w

        # Convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion([q_x, q_y, q_z, q_w])

        # Publish yaw value
        yaw_degrees =int(-math.degrees(yaw))#(180- int(math.degrees(yaw)))+53
        if yaw_degrees<0:
            yaw_degrees=yaw_degrees+360


        

        if yaw_degrees>360:
            yaw_degrees=360-yaw_degrees

        yaw_degrees=int(yaw_degrees)-108
        print(yaw_degrees)
        heading=Int32()
        heading.data=int(yaw_degrees)
        yaw_c=math.radians(yaw_degrees)

        self.imu_msg.header.stamp=rospy.Time.now()
        self.imu_msg.header.frame_id='imu_compass'
        x_c,y_c,z_c,w_c=quaternion_from_euler(roll,pitch,yaw_c)
        self.imu_msg.orientation.x=x_c
        self.imu_msg.orientation.y=y_c
        self.imu_msg.orientation.z=z_c
        self.imu_msg.orientation.w=w_c

        self.imu_pub.publish(self.imu_msg)
        self.heading_compass_pub.publish(heading)

if __name__ == '__main__':
    try:
        imu_to_yaw_converter = IMUToYawConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

