#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy                                         # Message type for Joystick
from sensor_msgs.msg import JointState                                  # Message type for State reading
from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.msg import JointPosition


class OpenManipulator_joy_commander:

    def __init__(self, ros_init=0):
        # VARIABLE TO SAVE INFO ON CALLBACKS
        self.commandLR = 0                      # Refers to LEFT/RIGHT
        self.commandUD = 0                      # Refers to UP/DOWN
        self.commandFINISH = 0                  # Refers to A buttom
        self.actualJoints = JointState()        # Refers to Actual Joint States values
        # JOINTS LIMITS
        self.LIMIT_MAX_J1 = 3.05424             # Max Limit for J1 based on URDF
        self.LIMIT_MIN_J1 = -3.05424            # Min Limit for J1 based on URDF
        self.LIMIT_MAX_J2 = 1.57075             # Max Limit for J2 based on URDF
        self.LIMIT_MIN_J2 = -1.57075            # Min Limit for J2 based on URDF
        # PARAMETERS
        self.stopCond = 0.01                    # Limit to compare target and actual state
        self.stepAxis = 0.15                    # How much will increase your axis movement
        # CONDITION - PREVENT ANY ROS INITIALIZE IF YOU IMPORT THIS OBJECT CLASS
        if ros_init:    
            # INIT THE NODE
            rospy.init_node('omp_joy_commander')
            # ROS LOOP REGULARIZER :: [Hz] frequency to send a message
            self.rate = rospy.Rate(2)                  
            # DEFINE A SERVICE TO COMMAND YOUR ROBOT JOINTS
            topic_commands = '/open_manipulator_pro/goal_joint_space_path'              # Service proper name
            rospy.wait_for_service(topic_commands)                                      # Wait until the service is up
            self.SRV_commander = rospy.ServiceProxy(topic_commands, SetJointPosition)   # Get the service
            # DEFINE MESSAGE SCOPE TO OUR SERVICE
            self.msg_joint = JointPosition()                                            # Define your MSG type var
            self.msg_joint.joint_name = ['none']                                        # MSG - Joint Name defined as a list of all joints
            self.msg_joint.position = [999]                                             # MSG - Position value defined as a list
            self.msg_joint.max_accelerations_scaling_factor = 0.8                       # MSG - default limit acceleration
            self.msg_joint.max_velocity_scaling_factor = 0.8                            # MSG - default limit for arm speed
            self.arg_path_time = 1.0                                                    # PARAMETER - Path Planning
            self.arg_planning_group = ''                                                # PARAMETER - MoveIt! Planning group name
            # DEFINE A SUBSCRIBER for JOY COMMANDS
            rospy.Subscriber('/joy', Joy, self.callback, queue_size=10)
            # DEFINE A SUBSCRIBER for JOINT STATES
            rospy.Subscriber('/open_manipulator_pro/joint_states', JointState, self.callbackJS, queue_size=10)

    # CALLBACK - GET JOY DATA
    def callback(self, joyMsg):
        # GET JOYSTICK BUTTON UPPER +1 || DOWN -1
        self.commandUD = joyMsg.axes[7]
        # GET JOYSTICK BUTTON LEFT  +1 || RIGHT -1
        self.commandLR = joyMsg.axes[6]
        # GET JOYSTICK BUTTOM A
        self.commandFINISH = joyMsg.buttons
    
    # CALLBACK - GET JOINT STATES
    def callbackJS(self, jointState):
        self.actualJoints = jointState

    # FUNCTION - COMMAND GRIPPER
    def srv_joint_commander(self,list_positions):
        # SET MSG ARGUMENTS
        self.msg_joint.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5','joint6']
        self.msg_joint.position = list_positions
        # CALL SERVICE
        self.SRV_commander(self.arg_planning_group, self.msg_joint, self.arg_path_time)

    # METHOD - SEND OMP TO pHOME
    def goHome(self):
        # DEFINE VALUES TO SEND IN msg_joint
        poses = [-0.1, 0.0, -0.78, 0.0, 0.0,-1.84]
        # CALL SERVICE
        self.srv_joint_commander(list_positions=poses)
        # WAIT UNTIL ALL states ARE equally to target
        while not rospy.is_shutdown():
            stopper = 0
            if len(self.actualJoints.position) != 0:
                for i in range(0, len(poses)):
                    if abs(abs(poses[i]) - abs(self.actualJoints.position[i])) <= self.stopCond:
                        stopper += 1
                if stopper == len(poses):
                    rospy.loginfo('pHOME Finished!')
                    break

    # METHOD - COMMANDS OMP WITH JOYSTICK
    def joyCommandsOMP(self):
        rospy.loginfo('XBOX 360 JOY COMMANDER READY!')
        # LOOP UNTIL THIS NODE IS UP
        while not rospy.is_shutdown():
            # GET ACTUAL VALUES
            jointsValues = list(self.actualJoints.position)[0:6]
            # JOINT 1
            if self.commandLR == 1:
                #rospy.loginfo('+X >> +J1')
                jointsValues[0] += self.stepAxis                # INSERT STEP
                if jointsValues[0] < self.LIMIT_MAX_J1:         # MOVE ONLY IF
                    self.srv_joint_commander(jointsValues)

            elif self.commandLR == -1:
                #rospy.loginfo('-X >> -J1')
                jointsValues[0] -= self.stepAxis
                if jointsValues[0] > self.LIMIT_MIN_J1:
                    self.srv_joint_commander(jointsValues)
            # JOINT 2
            elif self.commandUD == 1:
                #rospy.loginfo('+Y >> +J2')
                jointsValues[1] += self.stepAxis           # INSERT STEP
                if jointsValues[1] < self.LIMIT_MAX_J2:    # MOVE ONLY IF
                    self.srv_joint_commander(jointsValues)
            elif self.commandUD == -1:
                #rospy.loginfo('-Y >> -J2')
                jointsValues[1] -= self.stepAxis
                if jointsValues[1] > self.LIMIT_MIN_J2:
                    self.srv_joint_commander(jointsValues)
            
            # BREAK CONDITION - Press A
            if self.commandFINISH == 1:
                break
            # KEEP LOOP TIMING
            self.rate.sleep()

# MAIN
if __name__ == '__main__':
    try:
        jc = OpenManipulator_joy_commander(ros_init=1)
        jc.goHome()
        jc.joyCommandsOMP()

    except rospy.ROSInterruptException:
        pass
