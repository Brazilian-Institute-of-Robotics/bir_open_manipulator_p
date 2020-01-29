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
ROTATE_DEGREE = -pi*30/180                      # ROTATE -10 DEGREES
LIMIT_J1 = -3.05                                # ROTATION LIMIT FOR JOINT 1
END_CONDITION = 0.20                            # END CONDITION FOR SEARCHING
SEQ_FRAMES = 100                                # HOW MANY FRAME DO YOU NEED TO COUNT TO FIND THE DESIRED TAG

class openManipulator:
    def __init__(self):
        # ROS NODE INIT
        rospy.init_node('apriltag_detector_go2goal_gripper', anonymous=True)        
        # MOVEIT INIT
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "arm"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
        # TAG DETECTION INIT
        self.tag_id_subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray,self.callback)
        # GRIPPER SERVICE INIT
        rospy.wait_for_service('/open_manipulator_pro/goal_tool_control') 
        self.gripper_commander = rospy.ServiceProxy('/open_manipulator_pro/goal_tool_control',SetJointPosition)   
        self.gripper_msg = JointPosition()
        self.gripper_msg.joint_name = ['gripper']
        self.gripper_msg.max_accelerations_scaling_factor = 0.1
        self.gripper_msg.max_velocity_scaling_factor = 0.5
        self.gripper_msg.position = [2]                      # START OPEN                                                    
        # DEFINE MOVEIT PARAMETERS
        self.group.set_goal_position_tolerance(0.1)          # GOAL TOLERANCE
        self.group.set_max_velocity_scaling_factor(0.1)      # MAX SPEED FACTOR
        self.group.set_planning_time(5)                      # TIME TO PLANNING
        # DEFINE TAG SEARCH AUX
        self.id_found = 0                                    # FOUND TAG
        self.seq_counts = 0                                  # SEQ COUNTS THAT THE RBG GET THE FRAME
        self.init_pose = 0                                   # STOP FIND TAG BEFORE START ROUTINE

    # FUNCTION - GO TO SPECIFIC POSE
    def go_to_pose(self, pose_name):
        ## WE CAN PLAN AND EXECUTE A MOTION FOR THIS GROUP TO A DESIRED SAVED POSE FOR THE END-EFF
        # 1 - PASS YOUR POSE SAVED ON SETUP ASSISTANT
        self.group.set_named_target(pose_name)
        # 2 - PLAN AND EXECUTE
        self.group.go(wait=True)
        # 3 - PREVENT RESIDUAL MOVEMENT
        self.group.stop()
        # 4 - CLEAR TARGET GOAL
        self.group.clear_pose_targets()

    # FUNCTION - ROTATE JOINT 1 LOOKING FOR A TAG
    def search_tag(self):
        # 1 - DEFINE A VARIABLE FORMAT EQUIVALENT TO JOINTS
        joint_goal = self.group.get_current_joint_values()
        # 2 - DECISION :: FINISH THE NODE IF JOINT ANGLE IS CLOSE TO PATH END
        if abs(abs(joint_goal[0]) - abs(LIMIT_J1)) < END_CONDITION:
            # 2.1 - BEFORE END NODE, SEND TO pSearch
            rospy.loginfo('############# GO TO HOME POSE, FINISH ROUTINE.')
            self.go_to_pose('pHome')
            rospy.sleep(10)
            sys.exit()
        # 3 - INSERT DESIRED ANGLE
        joint_goal[0] = joint_goal[0] + ROTATE_DEGREE
        # 4 - GO!
        self.group.go(joints=joint_goal, wait=True)
        # 6 - Print Joint state
        rospy.loginfo('Joint 1'+str(round(joint_goal[0]*180/pi,1))+' degrees'+' | '+ str(joint_goal[0])+' rad')
        # 7 - STOP  ANY RESIDUAL MOVEMENT
        self.group.stop()
        # 8 - CLEAR TARGET GOAL
        self.group.clear_pose_targets()

    # FUNCTION - SET SPECIFIC JOINT
    def set_joint_and_moveit(self, jointIndex, jointAngle):
        # TRANSFORMATION
        jointIndex = jointIndex - 1           # TRANSLATE FROM 1 TO 0
        #jointAngle = jointAngle*pi/180       # CONVERT TO RAD FROM DEGREE
        # 1 - DEFINE A VARIABLE FORMAT EQUIVALENT TO JOINTS
        joint_goal = self.group.get_current_joint_values()
        # 2 - INSERT DESIRED ANGLE
        joint_goal[jointIndex] = jointAngle
        # 3 - GO!
        self.group.go(joints=joint_goal, wait=True)
        # 4 - STOP  ANY RESIDUAL MOVEMENT
        self.group.stop()
        # 5 - CLEAR TARGET GOAL
        self.group.clear_pose_targets()                

    # FUNCTION - SET POSE MANUALLY
    def set_pose(self):
        # TRANSFORMATION
        # 1 - DEFINE A VARIABLE FORMAT EQUIVALENT TO JOINTS
        joint_goal = self.group.get_current_joint_values()
        # 2 - INSERT DESIRED ANGLEs
        joint_goal[0] = -2.8273
        joint_goal[1] = -0.2792 
        joint_goal[2] = 1.0470
        joint_goal[4] = 0
        # 3 - GO!
        self.group.go(joints=joint_goal, wait=True)
        # 4 - STOP  ANY RESIDUAL MOVEMENT
        self.group.stop()
        # 5 - CLEAR TARGET GOAL
        self.group.clear_pose_targets()                

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
            
    # FUNCTION - PROCESS ROUTINE
    def process(self):
        ## STEP 1 - OPEN GRIPPER
        #self.go_to_pose('pHome')
        #rospy.sleep(20)
        self.go_gripper(2)
        rospy.sleep(1.5)
        ## STEP 2 - GO TO SEARCH ZONE
        self.go_to_pose('pSearch')
        rospy.sleep(15)
        self.init_pose = 1
        ## SOLVER TAG ISSUE BY RESTARTING VARIABLE
        self.id_found = 0                                 
        # STEP 3 - SEARCH THE TAG
        while not rospy.is_shutdown():
            # 3.1 - IF TAG WAS NOT FOUND, ROTATE
            if self.id_found is 0:
                self.search_tag()
                rospy.loginfo('############# ROTATION COMPLETE')
                rospy.sleep(8)
                
            # 3.2 - IF TAG WAS FOUND, GO TO TARGET POSITION
            elif self.id_found is 1:
                rospy.loginfo('############# TAG FOUND')
                # GO TO CATCH POSITION
                self.set_pose()
                rospy.sleep(15)
                # PREPARE TO CATCH
                self.set_joint_and_moveit(5, 0.9598)
                rospy.sleep(6)
                # CATCH WITH GRIPPER
                self.go_gripper(0.7)
                rospy.sleep(1.5)
                self.set_joint_and_moveit(5, 0.0)
                rospy.sleep(6)
                # FINISH PROGRAM
                sys.exit()   

    # AUXILIAR FUNCTION FOR SEVERAL TESTS
    def process2(self):
        self.go_gripper(2)
        rospy.sleep(1.5)
        #self.go_to_pose('pSearch')
        #rospy.sleep(15)
        #self.set_pose()
        #rospy.sleep(15)
        self.set_joint_and_moveit(5, 0.9598)
        rospy.sleep(15)
        self.go_gripper(0.7)
        rospy.sleep(1.5)
        self.set_joint_and_moveit(5, 0)
        rospy.sleep(6)               
        #self.go_gripper(2)
        #rospy.sleep(1.5)
        #self.go_gripper(0.6)
        #rospy.sleep(1.5)
        #self.go_gripper(2)
        #rospy.sleep(1.5)


if __name__ == '__main__':
    try:
        # INITIALIZE YOUR OBJECT
        omanip = openManipulator()
        # INITIALIZE THE OPERATION BY CLICKING ENTER
        raw_input("Press Enter to start!")
        # OPERATION
        omanip.process()  

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass       