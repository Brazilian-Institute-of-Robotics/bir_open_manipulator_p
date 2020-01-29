#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray
from moveit_commander.conversions import pose_to_list
from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.msg import JointPosition

# PARAMETERS
JOINTS_TOLERANCE = 0.01
SPEED_FACTOR = 0.8


class openManipulatorBIR:
    def __init__(self):
        # ROS NODE INIT
        rospy.init_node('omp_moveit_commander')        
        # MOVEIT INIT
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
        # GRIPPER SERVICE INIT
        rospy.wait_for_service('/open_manipulator_pro/goal_tool_control') 
        self.gripper_commander = rospy.ServiceProxy('/open_manipulator_pro/goal_tool_control',SetJointPosition)   
        self.gripper_msg = JointPosition()
        self.gripper_msg.joint_name = ['gripper']
        self.gripper_msg.max_accelerations_scaling_factor = 0.1
        self.gripper_msg.max_velocity_scaling_factor = 0.5
        self.gripper_msg.position = [2]                                   # START OPEN                                                    
        # DEFINE MOVEIT PARAMETERS
        self.group.set_goal_position_tolerance(JOINTS_TOLERANCE)          # GOAL TOLERANCE
        self.group.set_max_velocity_scaling_factor(0.1)                   # MAX SPEED FACTOR
        self.group.set_planning_time(5)                                   # TIME TO PLANNING
        # DEFINE TAG SEARCH AUX
        self.id_found = 0                                    # FOUND TAG
        self.seq_counts = 0                                  # SEQ COUNTS THAT THE RBG GET THE FRAME
        self.init_pose = 0                                   # STOP FIND TAG BEFORE START ROUTINE

    # FUNCTION - GO TO POSE WITH SECURITY
    def security_sequential_execution2(self, pose_name, numberOfJoints=6):
        # SET TARGET & GO
        if not rospy.is_shutdown():
            self.group.set_named_target(pose_name)
            self.group.go(wait=False)
            # GET TARGET VALUES
            jointsTarget = self.group.get_joint_value_target()
        # VERIFY IF WE GET CLOSER TO OUR TARGET
        while not rospy.is_shutdown():
            jointsActual = self.group.get_current_joint_values()    # Joint State values
            stopCondition = 0                                       # Count Vars
            # LOOP TO VERIFIY :: |Actual - Target| < Tolerance
            for i in range(0, numberOfJoints):
                if abs(abs(jointsActual[i])-abs(jointsTarget[i])) <= JOINTS_TOLERANCE:
                    stopCondition += 1
            # STOP CONDITION
            if stopCondition == numberOfJoints:
                self.group.stop()
                self.group.clear_pose_targets()
                rospy.sleep(1)
                break
    
    # FUNCTION - MOVE JOINT
    # FUNCTION - GO TO POSE WITH SECURITY
    def move_joint_security_sequential_execution(self, jointID, jointValueDegree):
        # FIX PARAMETERS
        jointID = jointID - 1
        jointValueRad = jointValueDegree*pi/180
        # GET ACTUAL STATE
        joint_goal = self.group.get_current_joint_values()
        # SET YOUR VARIATION
        joint_goal[jointID] = jointValueRad
        # GO!
        self.group.go(joints=joint_goal, wait=False)
        # LOOP TO VERIFIY :: |Actual - Target| < Tolerance
        while not rospy.is_shutdown():
            # GET ACTUAL STATE TO VERIFY
            joint_actual = self.group.get_current_joint_values()
            # STOP CONDITION
            if abs(abs(joint_goal[jointID]) - abs(joint_actual[jointID])) <= JOINTS_TOLERANCE:
                self.group.stop()
                self.group.clear_pose_targets()
                rospy.sleep(1)
                break

    # FUNCTION - ROTATE JOINT 1 LOOKING FOR A TAG
    def search_tag(self):
        # 1 - DEFINE A VARIABLE FORMAT EQUIVALENT TO JOINTS
        joint_goal = self.group.get_current_joint_values()
        # 2 - DECISION :: FINISH THE NODE IF JOINT ANGLE IS CLOSE TO PATH END
        if abs(abs(joint_goal[0]) - abs(LIMIT_J1)) < END_CONDITION:
            # 2.1 - BEFORE END NODE, SEND TO pSearch
            rospy.loginfo('############# GO TO HOME POSE, FINISH ROUTINE.')
            self.security_sequential_execution2('pHome')
            rospy.sleep(10)
            sys.exit()
        # 3 - INSERT DESIRED ANGLE
        joint_goal[0] = joint_goal[0] + ROTATE_DEGREE
        # 4 - GO!
        self.group.go(joints=joint_goal, wait=True)
        # 6 - Print Joint state
        rospy.loginfo('Joint 1'+str(round(joint_goal[0]*180/pi,1))+' degrees'+' | '+ str(joint_goal[0])+' rad')
        while not rospy.is_shutdown():
            jointsActual = self.group.get_current_joint_values()    # Joint State values
            if abs(abs(jointsActual[0])-abs(joint_goal[0])) <= 0.1:
                self.group.stop()
                self.group.clear_pose_targets()
                rospy.sleep(1)
                break                  

    # FUNCTION - CALLBACK
    def callback(self,data):
        if self.id_found is not 1:
            if len(data.detections) is not 0:
                if data.detections[0].id[0] == 1 and self.init_pose == 1:
                    self.seq_counts = self.seq_counts + 1
                else:
                    self.seq_counts = 0
        
                if self.seq_counts == SEQ_FRAMES and self.init_pose == 1:
                    self.id_found = 1
                    rospy.loginfo('TAG FOUND')
        else:
            pass

    # FUNCTION - COMMAND GRIPPER
    def go_gripper(self, positionValue):
        self.gripper_msg.position = [positionValue]
        self.gripper_commander('arm',self.gripper_msg,5.0)