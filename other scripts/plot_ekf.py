#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class PositionPlotter:
    def __init__(self):
        rospy.init_node('position_plotter', anonymous=True)
        
        self.wheel_odom_sub = rospy.Subscriber('/odom_ticks', Odometry, self.wheel_odom_callback)
        self.visual_odom_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.visual_odom_callback)
        self.robo_local_call = rospy.Subscriber('/odom_filtered', Odometry, self.robo_localis_callback)

        self.pose_covariance_sub = rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.pose_covariance_callback)

        self.x_data_wheel = []
        self.y_data_wheel = []
        self.x_data_vis = []
        self.y_data_vis = []
        self.x_data_ekf = []
        self.y_data_ekf = []
        self.x_data_loc = []
        self.y_data_loc = []
        self.legend_labels = ['wheel_odom', 'Visual Odometry','robot_pose_ekf','robot_localisation']

        # plt.ion()  # Turn on interactive mode for plotting
        # self.fig, self.ax = plt.subplots()
        # self.ax.set_xlabel('X Position')
        # self.ax.set_ylabel('Y Position')
        # self.ax.set_title('Robot Position')
        # self.legend = self.ax.legend(self.legend_labels)
        # self.plot, = self.ax.plot([], [], 'o')
        # self.fig.canvas.draw()

    def robo_localis_callback(self, msg):
       x,y=msg.pose.pose.position.x, msg.pose.pose.position.y
       self.x_data_loc.append(x)
       self.y_data_loc.append(y)

    def wheel_odom_callback(self, msg):
       x,y=msg.pose.pose.position.x, msg.pose.pose.position.y
       self.x_data_wheel.append(x)
       self.y_data_wheel.append(y)

    def visual_odom_callback(self, msg):
        x,y=msg.pose.pose.position.x, msg.pose.pose.position.y
        self.x_data_vis.append(x)
        self.y_data_vis.append(y)
        
    def pose_covariance_callback(self, msg):
        x,y=msg.pose.pose.position.x, msg.pose.pose.position.y
        self.x_data_ekf.append(x)
        self.y_data_ekf.append(y)

    # def process_odometry(self, x, y):
    #     self.x_data.append(x)
    #     self.y_data.append(y)

    #     self.plot.set_xdata(self.x_data)
    #     self.plot.set_ydata(self.y_data)
    #     self.ax.relim()
    #     self.ax.autoscale_view()
    #     self.fig.canvas.draw()

    def run(self):
        plt.figure()
        plt.title('Robot Position')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        
        while not rospy.is_shutdown():

            plt.plot(self.y_data_wheel,self.x_data_wheel,'b-')
            plt.plot(self.y_data_vis,self.x_data_vis,'g-')
            plt.plot(self.y_data_ekf,self.x_data_ekf,'r-')
            plt.plot(self.y_data_loc,self.x_data_loc,'y-')

            plt.legend(self.legend_labels)
            plt.xlim(-100,30)  # Adjust these limits based on your scenario
            plt.ylim(-10, 80)
            plt.pause(0.1)
            plt.clf()

        plt.show()


if __name__ == '__main__':
    try:
        position_plotter = PositionPlotter()
        position_plotter.run()
    except rospy.ROSInterruptException:
        pass
