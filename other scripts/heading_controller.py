#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import math
import time


def euler_from_quaternion(
    x,
    y,
    z,
    w,
    ):
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
    t2 = (+1.0 if t2 > +1.0 else t2)
    t2 = (-1.0 if t2 < -1.0 else t2)
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return (roll_x, pitch_y, yaw_z)


class heading_controller:

    def __init__(self):
        self.error = 0
        self.initial_heading = 0
        self.current_heading = 0

        rospy.init_node('headig_controller')
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.initial_heading = None
        self.current_hheading = 0
        self.heading_pub = rospy.Publisher('imu_heading', Float32, queue_size=10)
        self.rate=rospy.Rate(50)

    def imu_callback(self, data):

        (_, _, yaw_rad) = euler_from_quaternion(data.orientation.x,
                data.orientation.y, data.orientation.z,
                data.orientation.w)

        self.current_heading = math.degrees(yaw_rad)

        if self.initial_heading is None:
            self.initial_heading = self.current_heading

    def pwm_publisher(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            self.error = 0
            if self.initial_heading is not None:
                self.error = self.current_heading - self.initial_heading
#                print('error here ',self.error)
                if abs(self.error) > 50:
                    if self.error > 0:

                        self.error -= 360
                    else:
                        self.error += 360

#            print ('error ', self.current_heading,self.error)
#            time.sleep(0.1)
            heading=Float32()
            heading.data=math.radians(self.error)
            self.heading_pub.publish(heading)
            self.rate.sleep()
            


if __name__ == '__main__':
    head_contr = heading_controller()
    head_contr.pwm_publisher()

