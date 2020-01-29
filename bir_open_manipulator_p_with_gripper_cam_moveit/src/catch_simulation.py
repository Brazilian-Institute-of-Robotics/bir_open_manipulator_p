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
ROTATE_DEGREE = -pi*10/180                      # ROTATE -10 DEGREES
LIMIT_J1 = -3.05                                # ROTATION LIMIT FOR JOINT 1
END_CONDITION = 0.20                            # END CONDITION FOR SEARCHING
SEQ_FRAMES = 10                                 # HOW MANY FRAME DO YOU NEED TO COUNT TO FIND THE DESIRED TAG

class openManipulator:
    def __init__(self):
        # ROS NODE INIT
        rospy.init_node('gripper_get_with_tag', anonymous=True)        
        # MOVEIT INIT
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "arm"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
        # DEFINE TAG SEARCH AUX
        self.id_found = 0                                    # FOUND TAG
        self.seq_counts = 0                                  # SEQ COUNTS THAT THE RBG GET THE FRAME
        self.init_pose = 0                                   # STOP FIND TAG BEFORE START ROUTINE
        # TAG DETECTION INIT
        self.tag_id_subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray,self.callback)
        # DEFINE MOVEIT PARAMETERS
        self.group.set_goal_position_tolerance(0.5)          # GOAL TOLERANCE
        self.group.set_max_velocity_scaling_factor(1)        # MAX SPEED FACTOR
        self.group.set_planning_time(5)                      # TIME TO PLANNING

    # FUNCTION - SET TARGET & GO WITH PREVENTION
    def security_sequential_execution2(self, pose_name):
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
            for i in range(0, len(jointsActual)):
                if abs(abs(jointsActual[i])-abs(jointsTarget[i])) <= 0.1:
                    stopCondition += 1
            # STOP CONDITION
            if stopCondition == len(jointsActual):
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
            self.go_to_pose('pHome')
            sys.exit()
        # 3 - INSERT DESIRED ANGLE TO ROTATE
        joint_goal[0] = joint_goal[0] + ROTATE_DEGREE
        # 4 - GO!
        self.group.go(joints=joint_goal, wait=True)
        # 6 - PRINT ROTATION STATUS
        rospy.loginfo('JOINT 1'+str(round(joint_goal[0]*180/pi,1))+' DEGREES'+' | '+ str(joint_goal[0])+' rad.')
        # 7 - STOP  ANY RESIDUAL MOVEMENT
        self.group.stop()
        # 8 - CLEAR TARGET GOAL
        self.group.clear_pose_targets()

    # FUNCTION - SET SPECIFIC JOINT
    def set_joint_and_moveit(self, jointIndex, jointAngle):
        # TRANSFORMATION
        jointIndex = jointIndex - 1           # TRANSLATE TO i-1 INDEX
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
            
    # FUNCTION - PROCESS ROUTINE
    def process(self):
        ## STEP 1 - OPEN GRIPPER
        self.go_to_pose('pHome')
        ## STEP 2 - GO TO SEARCH ZONE
        self.go_to_pose('pSearch')
        ## 2.1 - INIT FLAGS
        self.init_pose = 1
        self.id_found = 0                                 
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
                self.go_to_pose('pCatch')
                # GO TO CATCH POSITION BOTTLE
                self.go_to_pose('pCatchBottle')
                # CATCH WITH GRIPPER
                self.set_joint_and_moveit(7, 1)
                # GET UP THE BOTTLE
                self.set_joint_and_moveit(5, -0.95)
                # FINISH PROGRAM
                sys.exit()   

    # AUXILIAR FUNCTION FOR SEVERAL TESTS
    def process2(self):
        self.go_to_pose('pHome')
        rospy.sleep(15)
        self.go_to_pose('pSearch')


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