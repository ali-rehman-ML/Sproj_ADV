#!/usr/bin/env python3

import time
import Jetson.GPIO as GPIO
import rospy
from std_msgs.msg import Int32

pin_rt_bl = 38
pin_rt_gr = 37
pin_lt_bl = 36
pin_lt_gr = 35
rostopic_ticks_rt_bl = "RWheel_bl"
rostopic_ticks_rt_gr = "RWheel_gr"
rostopic_ticks_lt_bl = "LWheel_bl"
rostopic_ticks_lt_gr = "LWheel_gr"

global ticks_rt_bl
global ticks_rt_gr
global ticks_lt_bl
global ticks_lt_gr

ticks_rt_bl = 0
ticks_rt_gr = 0 
ticks_lt_bl = 0
ticks_lt_gr = 0


def measure_ticks_per_second(pin):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    tick_count = 0
    start_time = time.time()
    elapsed_time = 0
    last_tick_time = start_time
    while elapsed_time < 0.3:
        current_state = GPIO.input(pin)

        if current_state == GPIO.HIGH:
            # If the sensor is activated
            if time.time() - last_tick_time > 0.01:  # Debounce time of 10ms
                tick_count += 1
                last_tick_time = time.time()
                
        elapsed_time = time.time() - start_time
    GPIO.cleanup()

    return tick_count

if __name__ == "__main__":
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    # Initialize ROS node
    rospy.init_node("robot_controller")
    # Create ROS publishers for ticks per second topics
    pub_ticks_rt_bl = rospy.Publisher(rostopic_ticks_rt_bl, Int32, queue_size=10)
    pub_ticks_rt_gr = rospy.Publisher(rostopic_ticks_rt_gr, Int32, queue_size=10)
    pub_ticks_lt_bl = rospy.Publisher(rostopic_ticks_lt_bl, Int32, queue_size=10)
    pub_ticks_lt_gr = rospy.Publisher(rostopic_ticks_lt_gr, Int32, queue_size=10)

    # Set the publishing rate
    rate = rospy.Rate(100)  # 1 Hz

    try:
        while not rospy.is_shutdown():

            ticks_rt_bl = measure_ticks_per_second(pin_rt_bl)
            ticks_rt_gr = measure_ticks_per_second(pin_rt_gr)
            ticks_lt_bl = measure_ticks_per_second(pin_lt_bl)
            ticks_lt_gr = measure_ticks_per_second(pin_lt_gr)

            print("left wheel", ticks_rt_bl)
            print("right wheel", ticks_lt_bl)
            
            # Publish ticks per second and cumulative ticks to ROS topics
            pub_ticks_rt_bl.publish(int(ticks_rt_bl))
            pub_ticks_rt_gr.publish(int(ticks_rt_gr))
            pub_ticks_lt_bl.publish(int(ticks_lt_bl))
            pub_ticks_lt_gr.publish(int(ticks_lt_gr))

            rate.sleep()

    except rospy.ROSInterruptException:
        GPIO.cleanup()
