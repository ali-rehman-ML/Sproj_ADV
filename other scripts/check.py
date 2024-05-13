#!/usr/bin/env python3

import time
import Jetson.GPIO as GPIO
import rospy
from std_msgs.msg import Int32

pin=38
GPIO.setmode(GPIO.BOARD)
GPIO.setup(pin, GPIO.IN,pull_up_down=GPIO.PUD_UP)
current_state = GPIO.input(pin)
count=0

low_check=True
high_check=True

#while(1):
#    current_state = GPIO.input(pin)
#    if current_state == GPIO.HIGH:
#        print("High")
#    

#    if current_state== GPIO.LOW:
#        print("Low")
#    time.sleep(0.1)


current_time_high=time.time()
previous_time_high=time.time()
current_time_low=time.time()
previous_time_low=time.time()
while(1):
    current_state = GPIO.input(pin)
    
    if current_state == GPIO.HIGH:
        current_time_high=time.time()
        if (current_time_high-previous_time_high)>0.080 and low_check==True:
            count=count+1
            previous_time_high=current_time_high
            low_check=False
            high_check=True

    if current_state== GPIO.LOW:
        current_time_low=time.time()
        if (current_time_low-previous_time_low)>0.080 and high_check==True:
            previous_time_low=current_time_low
            high_check=False
            low_check=True


    print(count)
#    time.sleep(0.018)

    
    
    
