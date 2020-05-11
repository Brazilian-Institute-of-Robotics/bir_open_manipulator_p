#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from math import pi
from apriltag_ros.msg import AprilTagDetectionArray
from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.msg import JointPosition

# CONSTANTS
N_ROBOT_JOINTS = 6
POSE_TOLERANCE = 0.05
FRAMES_LIMIT = 25
ROTATION_DEGREE = -10
J1_LIMIT_DEGREE = -90

class openManipulatorPRO:
    def __init__(self):
        # ROS NODE INIT
        rospy.init_node('omp_gripper_moveit_commander')
        # TAG DETECTION - VARS
        self.tag_found = 0                                     # FOUND TAG
        self.init_pose = 0                                     # STOP FIND TAG BEFORE START ROUTINE
        self.frames_count = 0                                  # Count frames 
        # MOVEIT - INIT
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")
        # GRIPPER SERVICE INIT
        rospy.wait_for_service('/open_manipulator_p/goal_tool_control') 
        self.gripper_commander = rospy.ServiceProxy('/open_manipulator_p/goal_tool_control',SetJointPosition)   
        self.gripper_msg = JointPosition()
        self.gripper_msg.joint_name = ['gripper']
        self.gripper_msg.max_accelerations_scaling_factor = 0.1
        self.gripper_msg.max_velocity_scaling_factor = 0.5
        self.gripper_msg.position = [2]                                               
        # TAG DETECTION - INIT
        self.tag_id_subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tagCB)

    # FUNCTION - COMMAND GRIPPER
    def set_gripper(self, positionValue):
        self.gripper_msg.position = [positionValue]
        self.gripper_commander('arm', self.gripper_msg, 5.0)

    # FUNCTION - CALLBACK
    def tagCB(self,data):
        if (self.tag_found is not 1) and (len(data.detections) is not 0):
            if data.detections[0].id[0] == 1 and self.init_pose == 1:
                self.frames_count += 1
            else:
                self.seq_counts = 0
        
            if self.frames_count == FRAMES_LIMIT and self.init_pose == 1:
                self.tag_found = 1
        else:
            pass

    # FUNCTION - GO TO POSE WITH SECURITY
    def go_to_pose_sequential_execution(self, pose_name):
        # SET TARGET & GO
        if not rospy.is_shutdown():
            self.group.set_named_target(pose_name)
            self.group.go(wait=False)
            jointsTarget = self.group.get_joint_value_target()
        # VERIFY IF WE GET CLOSER TO OUR TARGET
        while not rospy.is_shutdown():
            jointsActual = self.group.get_current_joint_values()    # Joint State values
            stopCondition = 0                                       # Count Vars
            # LOOP TO VERIFIY :: |Actual - Target| < Tolerance
            for i in range(0, N_ROBOT_JOINTS):
                if abs(jointsActual[i] - jointsTarget[i]) <= POSE_TOLERANCE:
                    stopCondition += 1
            # STOP CONDITION
            if stopCondition == N_ROBOT_JOINTS:
                self.group.stop()
                self.group.clear_pose_targets()
                rospy.sleep(0.5)
                break

    # FUNCTION - MOVE JOINT WITH SECURITY
    def move_joint_sequential_execution(self, jointID, jointValueDegree):
        # FIX PARAMETERS
        jointID = jointID - 1
        jointValueRad = jointValueDegree*pi/180
        # GET ACTUAL STATE & SET YOUR GOAL
        jointsTarget = self.group.get_current_joint_values()
        jointsTarget[jointID] = jointValueRad
        # GO!
        self.group.go(joints=jointsTarget, wait=False)
        # LOOP TO VERIFIY :: |Actual - Target| < Tolerance
        while not rospy.is_shutdown():
            # GET ACTUAL STATE TO VERIFY
            jointActual = self.group.get_current_joint_values()
            # STOP CONDITION
            if abs(jointActual[jointID] - jointsTarget[jointID]) <= POSE_TOLERANCE:
                self.group.stop()
                self.group.clear_pose_targets()
                rospy.sleep(0.5)
                break

    # FUNCTION - ROTATE JOINT 1 LOOKING FOR A TAG
    def search_tag(self):
        # DEFINE A VARIABLE FORMAT EQUIVALENT TO JOINTS
        jointTarget = self.group.get_current_joint_values()
        # VERIFY IF THE TARGET DEGREE IS HIGHER THAN J1 LIMIT
        if (jointTarget[0]*pi/180) + ROTATION_DEGREE <= J1_LIMIT_DEGREE:
            rospy.loginfo('J1 Limit Exit!')
            sys.exit()
        # SEARCH TAG
        else:
            jointTarget[0] = jointTarget[0] + (ROTATION_DEGREE*pi/180)
            self.group.go(joints=jointTarget, wait=True)
            while not rospy.is_shutdown():
                # GET ACTUAL STATE TO VERIFY
                jointActual = self.group.get_current_joint_values()
                # STOP CONDITION
                if abs(jointActual[0] - jointTarget[0]) <= POSE_TOLERANCE:
                    self.group.stop()
                    self.group.clear_pose_targets()
                    rospy.sleep(0.5)
                    break

    # FUNCTION - PROCESS ROUTINE
    def detect_and_get_routine(self):
        ## STEP 1 - OPEN GRIPPER
        self.set_gripper(2)
        rospy.sleep(0.5)
        ## STEP 2 - GO TO SEARCH HEIGTH ZONE
        self.go_to_pose_sequential_execution(pose_name='pSearch')
        ## START TAG VARIABLES
        self.tag_found = 0
        self.init_pose = 1                                 
        # STEP 3 - SEARCH THE TAG
        while not rospy.is_shutdown():
            # 3.1 - IF TAG WAS NOT FOUND, ROTATE
            if self.tag_found is 0:
                self.search_tag()
                
            # 3.2 - IF TAG WAS FOUND, GO TO TARGET POSITION
            elif self.tag_found is 1:
                rospy.loginfo('############# TAG FOUND')
                # GO TO CATCH POSITION
                self.go_to_pose_sequential_execution(pose_name='pCatch')
                # PREPARE TO CATCH
                self.go_to_pose_sequential_execution(pose_name='pCatchBottle')
                # CATCH WITH GRIPPER
                self.set_gripper(0.75)
                rospy.sleep(0.5)
                # GO TO PSEARCH
                self.move_joint_sequential_execution(jointID=3, jointValueDegree=35)
                self.go_to_pose_sequential_execution(pose_name='pFinish')
                ## STEP 1 - OPEN GRIPPER
                self.set_gripper(2)
                rospy.sleep(0.5)                
                # FINISH PROGRAM
                sys.exit()   

    # AUXILIAR FUNCTION FOR SEVERAL TESTS
    def test_routine(self):
        self.set_gripper(2)
        rospy.sleep(0.5)
        self.go_to_pose_sequential_execution(pose_name='pHome')
        self.go_to_pose_sequential_execution(pose_name='pSearch')
        self.go_to_pose_sequential_execution(pose_name='pCatch')
        self.go_to_pose_sequential_execution(pose_name='pCatchBottle')
        self.set_gripper(0.75)
        rospy.sleep(0.5)
        self.move_joint_sequential_execution(jointID=3, jointValueDegree=35)
        self.go_to_pose_sequential_execution(pose_name='pFinish')
        self.set_gripper(2)
        rospy.sleep(0.5)
	self.go_to_pose_sequential_execution(pose_name='pHome')

if __name__ == '__main__':
    try:
        # INITIALIZE YOUR OBJECT
        omanip = openManipulatorPRO()
        # OPERATION
        omanip.test_routine()  
	#omanip.detect_and_get_routine()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass       
