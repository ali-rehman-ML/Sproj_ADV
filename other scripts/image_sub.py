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
        # rospy.Subscriber("/camera/infra2/image_rect_raw", Image, self.callback2, queue_size=10)
        self.br = CvBridge()
        self.image1=None
        self.image2=None
        
        
    def callback(self,msg):
        self.image1=self.br.imgmsg_to_cv2(msg)
        # img=msg.data
        # print(msg.height,msg.width,msg.step,len(img))
    def callback2(self,msg):
        self.image2=self.br.imgmsg_to_cv2(msg)
        

    def run(self):
        
        while not rospy.is_shutdown(): 
            if self.image1 is not None:
                cv2.imshow("left ",self.image1)
                print(self.image1.shape)
            # cv2.imshow("right ",self.image2)
            
                cv2.waitKey(1)
            
            
        cv2.destroyAllWindows()
            
if __name__ == '__main__':
    imc=ImageConverter()
    time.sleep(0.5)
    imc.run()
    
        



