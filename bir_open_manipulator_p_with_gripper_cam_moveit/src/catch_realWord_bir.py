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


# CONSTANTS AND VARIABLES
ROTATE_DEGREE = -pi*25/180                      # ROTATE -25 DEGREES
LIMIT_J1 = -3.05                                # ROTATION LIMIT FOR JOINT 1
END_CONDITION = 0.20                            # END CONDITION FOR SEARCHING
SEQ_FRAMES = 15                                # HOW MANY FRAME DO YOU NEED TO COUNT TO FIND THE DESIRED TAG
JOINTS_TOLERANCE = 0.05

class openManipulatorPRO:
    def __init__(self):
        # ROS NODE INIT
        rospy.init_node('apriltag_detector_go2goal_gripper', anonymous=True)        
        # MOVEIT - INIT
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
        # MOVEIT - PARAMETERS
        self.group.set_goal_position_tolerance(JOINTS_TOLERANCE)            # GOAL TOLERANCE
        self.group.set_max_velocity_scaling_factor(0.95)                    # MAX SPEED FACTOR
        self.group.set_planning_time(5)                                     # TIME TO PLANNING
        # GRIPPER SERVICE INIT
        rospy.wait_for_service('/open_manipulator_pro/goal_tool_control') 
        self.gripper_commander = rospy.ServiceProxy('/open_manipulator_pro/goal_tool_control',SetJointPosition)   
        self.gripper_msg = JointPosition()
        self.gripper_msg.joint_name = ['gripper']
        self.gripper_msg.max_accelerations_scaling_factor = 0.1
        self.gripper_msg.max_velocity_scaling_factor = 0.5
        self.gripper_msg.position = [2]                                   # START OPEN                                                    
        # TAG DETECTION - INIT
        self.tag_id_subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray,self.callback)
        # TAG DETECTION - VARS
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

    # FUNCTION - MOVE JOINT WITH SECURITY
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
        # DEFINE A VARIABLE FORMAT EQUIVALENT TO JOINTS
        joint_goal = self.group.get_current_joint_values()
        # DECISION :: FINISH THE NODE IF JOINT ANGLE IS CLOSE TO PATH END
        if abs(abs(joint_goal[0]) - abs(LIMIT_J1)) < END_CONDITION:
            # 2.1 - BEFORE END NODE, SEND TO pSearch
            rospy.loginfo('############# GO TO HOME POSE, FINISH ROUTINE.')
            self.security_sequential_execution2('pHome')
            rospy.sleep(5)
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
        self.gripper_commander('arm', self.gripper_msg, 5.0)
            
    # FUNCTION - PROCESS ROUTINE
    def process(self):
        ## STEP 1 - OPEN GRIPPER
        self.go_gripper(2)
        rospy.sleep(0.5)
        ## STEP 2 - GO TO SEARCH HEIGTH ZONE
        self.security_sequential_execution2(pose_name='pSearch')
        ## SOLVER TAG ISSUE BY RESTARTING VARIABLE
        self.id_found = 0
        self.init_pose = 1                                 
        # STEP 3 - SEARCH THE TAG
        while not rospy.is_shutdown():
            # 3.1 - IF TAG WAS NOT FOUND, ROTATE
            if self.id_found is 0:
                self.search_tag()
                rospy.loginfo('############# ROTATION COMPLETE')
                
            # 3.2 - IF TAG WAS FOUND, GO TO TARGET POSITION
            elif self.id_found is 1:
                rospy.loginfo('############# TAG FOUND')
                # GO TO CATCH POSITION
                self.security_sequential_execution2(pose_name='pCatch')
                # PREPARE TO CATCH
                self.security_sequential_execution2(pose_name='pCatchBottle')
                # CATCH WITH GRIPPER
                self.go_gripper(0.8)
                rospy.sleep(2)
                # GO TO PSEARCH
                self.move_joint_security_sequential_execution(jointID=3, jointValueDegree=-52.5)
                self.security_sequential_execution2(pose_name='pSearch')
                ## STEP 1 - OPEN GRIPPER
                self.go_gripper(2)
                rospy.sleep(1.0)                
                # FINISH PROGRAM
                sys.exit()   

    # AUXILIAR FUNCTION FOR SEVERAL TESTS
    def process2(self):
        self.go_gripper(2)
        rospy.sleep(1.5)
        self.security_sequential_execution2(pose_name='pSearch')
        self.security_sequential_execution2(pose_name='pCatch')
        self.security_sequential_execution2(pose_name='pCatchBottle')
        self.go_gripper(0.8)
        rospy.sleep(1.5)
        self.move_joint_security_sequential_execution(jointID=3, jointValueDegree=-52.5)
        self.security_sequential_execution2(pose_name='pSearch')
        self.go_gripper(2)
        rospy.sleep(1.5)        

if __name__ == '__main__':
    try:
        # INITIALIZE YOUR OBJECT
        omanip = openManipulatorPRO()
        # INITIALIZE THE OPERATION BY CLICKING ENTER
        # OPERATION
        omanip.process2()  

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass       