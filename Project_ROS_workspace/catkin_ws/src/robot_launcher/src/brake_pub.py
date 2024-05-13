#! /usr/bin/env python
import rospy
from std_msgs.msg import Bool
import time

rospy.init_node('wheel_odometory_2')
class Brake:
    def __init__(self):
        rospy.init_node('wheel_odometory_2')
        self.obstacle_break_status=False
        self.controller_break_status=False

        self.obstacle_break_sub=rospy.Subscriber('/obstcale_brake',Bool,self.obstacle_break_callback)
        self.controller_break_sub=rospy.Subscriber('/controller_brake',Bool,self.controller_brake_callback)


        self.pub = rospy.Publisher('brake', Bool, queue_size=10)




    def obstacle_break_callback(self,msg):
        self.obstacle_break_status=bool(msg.data)
        print('data', self.obstacle_break_status)

    
    def controller_brake_callback(self,msg):
        self.controller_break_status=bool(msg.data)


    def publish(self):
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            print(self.obstacle_break_status,self.controller_break_status)
            brake=self.obstacle_break_sub or self.controller_break_status
            print("brake ", brake)
            if self.obstacle_break_status==True or self.controller_break_status==True:

                self.pub.publish(True)
            else:
                self.pub.publish(False)
                
            rate.sleep()

if __name__ == '__main__':
    br=Brake()
    time.sleep(0.5)
    br.publish()
    

