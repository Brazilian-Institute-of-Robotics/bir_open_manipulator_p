#!/usr/bin/env python
import rospy
import numpy as np
from math import pi as PI
import matplotlib.pyplot as plt # Plot library
from matplotlib.animation import FuncAnimation # Animation module
from geometry_msgs.msg import Pose
from itertools import count # Count Time
#from sensor_msgs.msg import JointState # Message type for State reading

class RealTimePlot():
    def __init__(self):
        # AUX VARS
        self.PERIOD_INFO = 500                                              # How much time in [ms] to get the next info and plot
        self.timer_count = count()                                          # Timer count to keep all variables at the same cycle/time
        self.fig, self.ax = plt.subplots(nrows=1, ncols=1)                  # Figure and axes to plot our real time data and use in class RTP
        # PLOT PARAMETERS
        self.fig.canvas.set_window_title('Real Time Plot - Time')                  # Define Window name
        # PLOT VARIABLES
        self.axis_command = []                                  # Time axis
        self.axis_x_commander = []                              # Controller axis X to plot 
        self.axis_y_commander = []                              # Controller axis Y to plot
        self.axis_z_commander = []                              # Controller axis Z to plot
        self.START_PLOT = 0
        # INIT NODE
        rospy.init_node('real_time_plotting_end_effector_time')
        # DEFINE SUBSCRIBER TO GET VALUES
        self.end_effector_subscriber =  rospy.Subscriber('/end_effector_axes_pose',
                                                        Pose, self.cb, queue_size=10)
    # CALLBACK - GET EE VALUES
    def cb(self, data):
        if data.orientation.x != 99.0:
            self.axis_x = data.position.x
            self.axis_y = data.position.y
            self.axis_z = data.position.z
            self.START_PLOT = 1

    # FUNCTION - ATUALIZE PLOT DATA WITH JOINTS VALUES in DEGREES
    def atualize_data(self, i):
        if not rospy.is_shutdown() and self.START_PLOT == 1:
            self.axis_command.append(next(self.timer_count)*self.PERIOD_INFO/1000)
            self.axis_x_commander.append(self.axis_x)
            self.axis_y_commander.append(self.axis_y)
            self.axis_z_commander.append(self.axis_z)
            # PLOT XYZ
            self.ax.cla()
            self.ax.set_title('Movimento da Ferramenta no plano XYZ', fontsize = 10)
            self.ax.set_ylabel('Valor [m]')
            self.ax.set_xlabel('Tempo [segundos]')
            self.ax.tick_params(labelsize=6)
            self.ax.plot(self.axis_command, self.axis_x_commander, label= "Eixo X")
            self.ax.plot(self.axis_command, self.axis_y_commander, label= "Eixo Y")
            self.ax.plot(self.axis_command, self.axis_z_commander, label= "Eixo Z")
            self.ax.legend(loc='lower right', ncol=1,  prop={"size":10})

    # FUNCTION - PLOT CONTINUOS DATA
    def plot_data(self):
        plt.style.use('fivethirtyeight')
        self.animation = FuncAnimation(self.fig, self.atualize_data, interval=self.PERIOD_INFO)
        #plt.tight_layout()
        plt.show()

# MAIN
if __name__ == '__main__':
    try:
        rtp = RealTimePlot()
        #rospy.spin()
        rtp.plot_data()
    except rospy.ROSInterruptException:
        pass        