#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
import time
from cv_bridge import CvBridge
import numpy as np
def imgmsg_to_cv2(img_msg, dtype=np.uint8):
    # it should be possible to determine dtype from img_msg.encoding but there is many different cases to take into account
    # original function args: imgmsg_to_cv2(img_msg, desired_encoding = "passthrough")
    return np.frombuffer(img_msg.data, dtype=dtype).reshape(img_msg.height, img_msg.width, -1)

class ImageConverter:

    def __init__(self):
        rospy.init_node('image_listener', anonymous=True)
        rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=10)
        self.pub = rospy.Publisher('/speed_breaker_brake', Bool, queue_size=10)

        self.br = CvBridge()
        self.image1=None
        
        
    def callback(self,msg):
        self.image1=self.br.imgmsg_to_cv2(msg)

        

    def run(self):
        
        while not rospy.is_shutdown(): 
            brake_status=False
            if self.image1 is not None and np.any(self.image1==4):
                image=cv2.resize(self.image1,(1280,720))
                height=np.where(image == 4)[0][-1]-np.where(image == 4)[0][0]
                dis=(0.30*643)/height
                if dis<5:
                    brake_status=True

            
            self.pub.publish(brake_status)
            
            
            
            
if __name__ == '__main__':
    imc=ImageConverter()
    time.sleep(0.5)
    imc.run()
    
        


