#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64        # Message type for OMP control in Gazebo
from sensor_msgs.msg import Joy         # Message type for Joystick
from sensor_msgs.msg import JointState  # Message type for State reading

class joy_commander:
    def __init__(self, ros_init=0):
        # VARIABLE TO SAVE INFO ON CALLBACKS
        self.commandX = 0                   # Refers to J2
        self.commandY = 0                   # Refers to J1
        self.commandFINISH = 0              # Refers to A buttom
        self.actualJoints = JointState()    # Refers to Actual Joint States values
        # PARAMETERS
        self.stopCond = 0.001               # Limit to compare MEAN(target - jointsState) to say if we got our destination
        self.stepAxis = 0.01                # How much will increase your axis movement
        # CONDITION TO PREVENT ANY ROS INIT IF YOU IMPORT THIS OBJECT CLASS
        if ros_init:    
            # INIT THE NODE
            rospy.init_node('joy_commander')
            # ROS LOOP REGULARIZER :: 10hz to send a message
            self.rate = rospy.Rate(10)                  
            # DEFINE A PUBLISHER for OMP JOINT 1 TOPIC
            self.pubJ1 = rospy.Publisher('/open_manipulator_p/joint1_position/command', Float64, queue_size=10)
            # DEFINE A PUBLISHER for OMP JOINT 2 TOPIC
            self.pubJ2 = rospy.Publisher('/open_manipulator_p/joint2_position/command', Float64, queue_size=10)
            # DEFINE A SUBSCRIBER for JOY COMMANDS
            self.sub = rospy.Subscriber('/joy', Joy, self.callback, queue_size=10)
            # DEFINE A SUBSCRIBER for JOINT STATES
            self.subJS = rospy.Subscriber('/open_manipulator_p/joint_states', JointState, self.callbackJS, queue_size=10)

    # METHOD - SET OMP IN HOME POSITION IN SIMULATION
    def ompHomePose(self):
        # DEFINE PUBLISHERS FOR OTHERS JOINTS
        pub3 = rospy.Publisher('/open_manipulator_p/joint3_position/command', Float64, queue_size=10)
        pub4 = rospy.Publisher('/open_manipulator_p/joint4_position/command', Float64, queue_size=10)
        pub5 = rospy.Publisher('/open_manipulator_p/joint5_position/command', Float64, queue_size=10)
        # DESIRED JOINTS POSITION (1 to 5)
        values = [-0.1, 0.0, -0.78, 0.0, 0.0]
        #values = [-0.5, 0.5, -0.50, 0.5, 0.5]
        # VALUE VAR TYPE
        value = Float64()
        # LOOP
        while not rospy.is_shutdown():
            # ROUTINE - Define Value & Send Message
            ## J1
            value.data = values[0]
            self.pubJ1.publish(value)
            ## J2
            value.data = values[1]
            self.pubJ2.publish(value)
            ## J3
            value.data = values[2]
            pub3.publish(value)
            ## J4
            value.data = values[3]
            pub4.publish(value)
            ## J5
            value.data = values[4]
            pub5.publish(value)
            ## BREAK CONDITION      
            if len(self.actualJoints.position) != 0:
                stop = 0
                for i in [0,1,2,3,4]:
                    if abs(abs(values[i]) - abs(self.actualJoints.position[i])) <= self.stopCond:
                        stop += 1
                if stop == 5:
                    rospy.loginfo('pHOME SUCCEED')
                    break
            # KEEP LOOP TIMING
            self.rate.sleep()

    # METHOD - CONTROLS OMP J1 & J2 WITH JOY
    def joystickCommandsRobot(self):
        rospy.loginfo('XBOX 360 JOY COMMANDER READY!')
        # INIT FORMAT TO PUBLISH IN JOINT COMMAND
        valueJ1 = Float64()
        valueJ2 = Float64()
        # START THE VALUES BASED IN ACTUAL JOINT STATES
        valueJ1.data = self.actualJoints.position[0]
        valueJ2.data = self.actualJoints.position[1]
        # UNTIL THIS NODE IS UP
        while not rospy.is_shutdown():
            # JOINT 1
            if self.commandX == 1:
                rospy.loginfo('+X >> +J1')
                valueJ1.data += self.stepAxis
                self.pubJ1.publish(valueJ1)

            elif self.commandX == -1:
                rospy.loginfo('-X >> -J1')
                valueJ1.data -= self.stepAxis
                self.pubJ1.publish(valueJ1)
            # JOINT 2
            if self.commandY == 1:
                rospy.loginfo('+Y >> +J2')
                valueJ2.data += self.stepAxis
                self.pubJ2.publish(valueJ2)
            elif self.commandY == -1:
                rospy.loginfo('-Y >> -J2')            
                valueJ2.data -= self.stepAxis
                self.pubJ2.publish(valueJ2)
            # BREAK CONDITION - Press A
            if self.commandFINISH == 1:
                break
            # KEEP LOOP TIMING
            self.rate.sleep()

    # CALLBACK - GET JOY DATA
    def callback(self, joyMsg):
        # GET JOYSTICK BUTTON UPPER +1 || DOWN -1
        self.commandY = joyMsg.axes[7]
        # GET JOYSTICK BUTTON LEFT  +1 || RIGHT -1
        self.commandX = joyMsg.axes[6]
        # GET JOYSTICK BUTTOM A
        self.commandFINISH = joyMsg.buttons
    
    # CALLBACK - GET JOINT STATES
    def callbackJS(self, jointState):
        self.actualJoints = jointState

# MAIN
if __name__ == '__main__':
    try:
        jc = joy_commander(ros_init=1)
        jc.ompHomePose()
        jc.joystickCommandsRobot()
    except rospy.ROSInterruptException:
        pass