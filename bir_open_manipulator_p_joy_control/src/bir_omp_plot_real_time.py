#!/usr/bin/env python
import rospy
import numpy as np
from math import pi as PI
import bir_omp_joystick_simulation as ompJS
import matplotlib.pyplot as plt                 # Plot library
from matplotlib.animation import FuncAnimation  # Animation module
from itertools import count                     # Count Time
from std_msgs.msg import Float64                # Message type for OMP control in Gazebo
from sensor_msgs.msg import Joy                 # Message type for Joystick
from sensor_msgs.msg import JointState          # Message type for State reading

class RealTimePlot():
    def __init__(self):
        # AUX VARS
        self.PERIOD_INFO = 550                                              # How much time in [ms] to get the next info and plot
        self.timer_count = count()                                          # Timer count to keep all variables at the same cycle/time
        self.previous_j1_state = -99                                        # Variable to keep previous J1 state
        self.previous_j2_state = -99                                        # Variable to keep previous J2 state 
        self.fig, self.ax = plt.subplots(nrows=2, ncols=2)                  # Figure and axes to plot our real time data and use in class RTP
        # PLOT PARAMETERS
        self.fig.canvas.set_window_title('Real Time Plot')                  # Define Window name
        # USEFUL VARIABLES
        self.communicate = ompJS.joy_commander(ros_init=0)      # Importing Methods/variables defined in ompJS
        self.axis_time = []                                     # Time axis
        self.axis_x_commander = []                              # Controller axis X to plot 
        self.axis_y_commander = []                              # Controller axis Y to plot 
        self.arm_j1_responder = []                              # Arm J1 response to plot
        self.arm_j2_responder = []                              # Arm J2 response to plot
        # INIT NODE
        rospy.init_node('real_time_plotting')
        # SUBSCRIBER - for JOY COMMANDS
        self.sub = rospy.Subscriber('/joy', Joy, self.communicate.callback, queue_size=10)
        # SUBSCRIBER - for JOINT STATES
        self.subJS = rospy.Subscriber('/open_manipulator_p/joint_states', JointState, self.communicate.callbackJS, queue_size=10)

    # FUNCTION - RETURN JOINT STATES in BOOLEAN VALUE
    def logic_joints(self, previous, actual):
        # Logic to define returns to plot
        if round(actual,3) > round(previous,3) and previous != -99:
            return 1, actual
        elif round(actual,3) < round(previous,3) and previous != -99:
            return -1, actual
        else:
            return 0, actual

    # FUNCTION - ATUALIZE PLOT DATA WITH DISCRETE VALUES [-1, 0, 1]
    def atualize_data_discrete(self, i):
        if not rospy.is_shutdown():
            # Axis Time
            self.axis_time.append(next(self.timer_count)*self.PERIOD_INFO/1000)
            # Axis X commander
            self.axis_x_commander.append(self.communicate.commandX)
            # Axis Y commander
            self.axis_y_commander.append(self.communicate.commandY)
            # CONDITION
            if len(self.communicate.actualJoints.position) != 0:
                # Arm J1 responder
                valueJ1, self.previous_j1_state = self.logic_joints(self.previous_j1_state, self.communicate.actualJoints.position[0])
                self.arm_j1_responder.append(valueJ1)
                # Arm J2 responder
                valueJ2, self.previous_j2_state = self.logic_joints(self.previous_j2_state, self.communicate.actualJoints.position[1])
                self.arm_j2_responder.append(valueJ2)

            # PLOTs
            ## Joystick X
            self.ax[1,0].cla()
            self.ax[1,0].set_title('Joystick X - LEFT(+1) & RIGHT(-1)', fontsize = 10)
            self.ax[1,0].set_ylabel('Direction')
            self.ax[1,0].set_yticks([-1, 0, 1])
            self.ax[1,0].set_xlabel('Time [micro seconds]')
            self.ax[1,0].tick_params(labelsize=6)
            self.ax[1,0].plot(self.axis_time, self.axis_x_commander)
            ## Joystick Y
            self.ax[1,1].cla()
            self.ax[1,1].set_title('Joystick Y - UP(+1) & DOWN(-1)', fontsize = 10)
            self.ax[1,1].set_ylabel('Direction')
            self.ax[1,1].set_yticks([-1, 0, 1])
            self.ax[1,1].tick_params(labelsize=6)
            self.ax[1,1].set_xlabel('Time [micro seconds]')
            self.ax[1,1].plot(self.axis_time, self.axis_y_commander)
            ## J1 Response (related to X)
            self.ax[0,0].cla()
            self.ax[0,0].set_title('Joint 1 - LEFT(+1) & RIGHT(-1)', fontsize = 10)
            self.ax[0,0].set_ylabel('Direction')
            self.ax[0,0].set_yticks([-1, 0, 1])
            self.ax[0,0].tick_params(labelsize=6)
            self.ax[0,0].set_xlabel('Time [micro seconds]')
            self.ax[0,0].plot(self.axis_time, self.arm_j1_responder)
            ## J2 Response (related to Y)
            self.ax[0,1].cla()
            self.ax[0,1].set_title('Joint 2 - UP(+1) & DOWN(-1)', fontsize = 10)
            self.ax[0,1].set_ylabel('Direction')
            self.ax[0,1].set_yticks([-1, 0, 1])
            self.ax[0,1].tick_params(labelsize=6)
            self.ax[0,1].set_xlabel('Time [micro seconds]')
            self.ax[0,1].plot(self.axis_time, self.arm_j2_responder)

    # FUNCTION - PLOT DISCRETE DATA
    def plot_data_discrete(self):
        plt.style.use('fivethirtyeight')
        self.animationDiscrete = FuncAnimation(self.fig, self.atualize_data_discrete, interval=self.PERIOD_INFO)
        plt.tight_layout()
        plt.show()

    # FUNCTION - ATUALIZE PLOT DATA WITH JOINTS VALUES in DEGREES
    def atualize_data(self, i):
        if not rospy.is_shutdown():
            # Axis Time
            self.axis_time.append(next(self.timer_count)*self.PERIOD_INFO/1000)
            # Axis X commander
            self.axis_x_commander.append(self.communicate.commandX)
            # Axis Y commander
            self.axis_y_commander.append(self.communicate.commandY)
            # CONDITION
            if len(self.communicate.actualJoints.position) != 0:
                ## Transform to degrees
                valueJ1 = round(self.communicate.actualJoints.position[0]*180/PI, 2)
                valueJ2 = round(self.communicate.actualJoints.position[1]*180/PI, 2)
                ## Arm J1 responder
                self.arm_j1_responder.append(valueJ1)
                ## Arm J2 responder
                self.arm_j2_responder.append(valueJ2)

            # PLOTs
            ## Joystick X
            self.ax[1,0].cla()
            self.ax[1,0].set_title('Joystick X - LEFT(+1) & RIGHT(-1)', fontsize = 10)
            self.ax[1,0].set_ylabel('Direction')
            self.ax[1,0].set_yticks([-1, 0, 1])
            self.ax[1,0].set_xlabel('Time [micro seconds]')
            self.ax[1,0].tick_params(labelsize=6)
            self.ax[1,0].plot(self.axis_time, self.axis_x_commander)
            ## Joystick Y
            self.ax[1,1].cla()
            self.ax[1,1].set_title('Joystick Y - UP(+1) & DOWN(-1)', fontsize = 10)
            self.ax[1,1].set_ylabel('Direction')
            self.ax[1,1].set_yticks([-1, 0, 1])
            self.ax[1,1].tick_params(labelsize=6)
            self.ax[1,1].set_xlabel('Time [micro seconds]')
            self.ax[1,1].plot(self.axis_time, self.axis_y_commander)
            ## J1 Response (related to X)
            self.ax[0,0].cla()
            self.ax[0,0].set_title('Joint 1 - LEFT(+) & RIGHT(-)', fontsize = 10)
            self.ax[0,0].set_ylabel('Direction [degrees]')
            self.ax[0,0].set_yticks([-175, 0, 175])
            self.ax[0,0].tick_params(labelsize=6)
            self.ax[0,0].set_xlabel('Time [micro seconds]')
            self.ax[0,0].plot(self.axis_time, self.arm_j1_responder)
            ## J2 Response (related to Y)
            self.ax[0,1].cla()
            self.ax[0,1].set_title('Joint 2 - UP(+) & DOWN(-)', fontsize = 10)
            self.ax[0,1].set_ylabel('Direction [degrees]')
            self.ax[0,1].set_yticks([-90, 0, 90])
            self.ax[0,1].tick_params(labelsize=6)
            self.ax[0,1].set_xlabel('Time [micro seconds]')
            self.ax[0,1].plot(self.axis_time, self.arm_j2_responder)

    # FUNCTION - PLOT CONTINUOS DATA
    def plot_data(self):
        plt.style.use('fivethirtyeight')
        self.animation = FuncAnimation(self.fig, self.atualize_data, interval=self.PERIOD_INFO)
        plt.tight_layout()
        plt.show()

# MAIN
if __name__ == '__main__':
    try:
        rtp = RealTimePlot()
        rtp.plot_data()
    except rospy.ROSInterruptException:
        pass        



    # atualize data

    # 






## GOAL: PUBLISH FROM JOY AND JOINT_STATES TOPIC
