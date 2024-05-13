#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from math import degrees, radians, sin,cos, atan2,atan
import  time
import matplotlib.pyplot as plt
class gps_odom_pub:
    def __init__(self):
        self.x=0.0
        self.y=0.0
        self.theta=0
        self.initial_heading=None
        self.Current_heading=None
        self.initi_lat=0.0
        self.initi_long=0.0
        self.curr_lat=0.0
        self.curr_long=0.0
        rospy.init_node('gps_odom', anonymous=True)

        rospy.Subscriber("/compass_heading", Int32, self.heading_call_back)
        rospy.Subscriber("/ublox/fix", NavSatFix, self.odom_call_back)
        self.curre_time=rospy.get_time()
        self.prev_time=rospy.get_time()
        
        
    def heading_call_back(self,msg):
        h=msg.data
        
        if self.initial_heading is None:
            self.initial_heading=h
            self.Current_heading=h
            
        else :
            self.Current_heading=h
            
        self.theta=self.Current_heading-self.initial_heading
        
        
    def odom_call_back(self,data):
	print("lat ", data.latitude, "long ", data.longitude)
        if self.initi_lat == 0.0:
            self.initi_lat=float(data.latitude)
            self.initi_long=float(data.longitude)
            self.curr_lat=float(data.latitude)
            self.curr_long=float(data.longitude)
        else :
            self.curr_lat=float(data.latitude)
            self.curr_long=float(data.longitude)

        self.transform()
        print(self.x,self.y,self.Current_heading)



    def transform(self):
        #print("here ", self.initi_lat,self.initi_long,self.curr_lat,self.curr_long)

        #if 31.0<self.curr_lat<32.0 and 70.0 <self.curr_long<76 and self.initi_lat>0 and self.initi_long>0:
        print("transform ", self.initi_lat,self.initi_long,self.curr_lat,self.curr_long)
        if self.initi_lat>0:
        
            tx=(self.curr_lat-self.initi_lat)*(111131)
            ty=(self.curr_long-self.initi_long)*(111131)
            # print("atan 2 ", degrees(atan(ty/tx)), degrees(atan(tx/ty)))
            # print(self.Current_heading)
            theta=radians(235)
        
            self.x=cos(theta)*tx +sin(theta)*ty
            self.y=-sin(theta)*tx + cos(theta)*ty


        else:
            pass
        
    def publish(self):
        plt.figure()
        plt.title('Robot Position')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        
        while not rospy.is_shutdown():

            plt.scatter(self.y, self.x, color='b')
            plt.xlim(-110,10)  # Adjust these limits based on your scenario
            plt.ylim(-5, 70)
            plt.pause(0.1)

        plt.show()
        plt.close('all')
            
            
            
if __name__ == '__main__':
    odom=gps_odom_pub()
    time.sleep(1)
    odom.publish()
            
