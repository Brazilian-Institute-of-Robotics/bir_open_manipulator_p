#!/usr/bin/env python
import sys
import rospy
from math import pi
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from apriltag_ros.msg import AprilTagDetectionArray



# CONSTANTS
N_ROBOT_JOINTS = 6
POSE_TOLERANCE = 0.01
FRAMES_LIMIT = 25
ROTATION_DEGREE = -20
J1_LIMIT_DEGREE = -175

class openManipulatorPRO:
    def __init__(self):
        # ROS NODE INIT
        rospy.init_node('OMP_gripper_moveit_commander') 
        # TAG DETECTION - VARS
        self.tag_found = 0                                     # FOUND TAG
        self.init_pose = 0                                     # STOP TO FIND TAG BEFORE START ROUTINE
        self.frames_count = 0                                  # FRAMES COUNT              
        # MOVEIT INIT
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory,queue_size=20)
        # TAG DETECTION INIT
        self.tag_id_subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray,self.tagCB)
        # MOVEIT RESTRICTIONS
        self.group.set_goal_position_tolerance(POSE_TOLERANCE)             # GOAL TOLERANCE
        self.group.set_planning_time(5)                         # TIME TO PLANNING

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

    # FUNCTION - SET SPECIFIC JOINT
    def set_joint_go(self, jointIndex, joint_angle_rad):
        # TRANSFORMATION
        jointIndex = jointIndex - 1           # TRANSLATE TO i-1 INDEX
        # 1 - DEFINE A VARIABLE FORMAT EQUIVALENT TO JOINTS
        joint_goal = self.group.get_current_joint_values()
        # 2 - INSERT DESIRED ANGLE
        joint_goal[jointIndex] = joint_angle_rad
        # 3 - GO!
        self.group.go(joints=joint_goal, wait=True)
        # 4 - STOP  ANY RESIDUAL MOVEMENT
        self.group.stop()
        # 5 - CLEAR TARGET GOAL
        self.group.clear_pose_targets()                
            
    # FUNCTION - PROCESS ROUTINE
    def detect_catch_routine(self):
        ## STEP 1 - OPEN GRIPPER
        self.go_to_pose('pHome')
        ## STEP 2 - GO TO SEARCH ZONE
        self.go_to_pose('pSearch')
        ## 2.1 - INIT FLAGS
        self.init_pose = 1
        self.tag_found = 0                                 
        # STEP 3 - SEARCH THE TAG
        while not rospy.is_shutdown():
            # 3.1 - IF TAG WAS NOT FOUND, ROTATE
            if self.tag_found is 0:
                self.search_tag()
                rospy.loginfo('############# ROTATION COMPLETE')
                
            # 3.2 - IF TAG WAS FOUND, GO TO TARGET POSITION
            elif self.tag_found is 1:
                rospy.loginfo('############# TAG FOUND')
                # GO TO CATCH POSITION
                self.go_to_pose('pCatch')
                # GO TO CATCH POSITION BOTTLE
                self.go_to_pose('pCatchBottle')
                # CATCH WITH GRIPPER
                self.set_joint_go(7, 1)
                # GET UP THE BOTTLE
                self.set_joint_go(5, -0.95)
                # FINISH PROGRAM
                sys.exit()   

    # AUXILIAR FUNCTION FOR SEVERAL TESTS
    def test_routine(self):
        self.go_to_pose('pHome')
        self.go_to_pose('pSearch')


if __name__ == '__main__':
    try:
        # INITIALIZE YOUR OBJECT
        omanip = openManipulatorPRO()
        # INITIALIZE THE OPERATION BY CLICKING ENTER
        raw_input("Press Enter to start!")
        # OPERATION
        omanip.detect_catch_routine()

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass       