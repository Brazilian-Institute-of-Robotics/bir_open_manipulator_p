#!/usr/bin/env python
import sys
import rospy
from math import pi
import moveit_commander
import geometry_msgs.msg
from moveit_msgs.msg import DisplayTrajectory
from apriltag_ros.msg import AprilTagDetectionArray

# CONSTANTS
N_ROBOT_JOINTS = 6
POSE_TOLERANCE = 0.05
FRAMES_LIMIT = 25
ROTATION_DEGREE = -20
J1_LIMIT_DEGREE = -175

class openManipulatorPRO:
    def __init__(self):
        # ROS NODE INIT
        rospy.init_node('OMP_moveit_commander')
        # TAG DETECTION - VARS
        self.tag_found = 0                                     # FOUND TAG
        self.init_pose = 0                                     # STOP TO FIND TAG BEFORE START ROUTINE
        self.frames_count = 0                                  # FRAMES COUNT         
        # MOVEIT INIT
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory,
                                                            queue_size=10)
        self.tag_id_subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray,self.tagCB)
        # MOVEIT RESTRICTIONS
        self.group.set_goal_position_tolerance(0.1)             # GOAL TOLERANCE
        self.group.set_planning_time(5)                         # TIME TO PLANNING

    # FUNCTION - PLAN AND EXECUTE A MOTION FOR THIS GROUP TO A DESIRED SAVED POSE FOR THE END-EFF
    def go_to_pose(self, pose_name):
        #  PASS YOUR POSE SAVED ON SETUP ASSISTANT
        self.group.set_named_target(pose_name)
        # PLAN AND EXECUTE
        self.group.go(wait=True)
        # PREVENT RESIDUAL MOVEMENT
        self.group.stop()
        # CLEAR TARGET GOAL
        self.group.clear_pose_targets()

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


    # FUNCTION - PROCESS ROUTINE
    def process(self):
        ## STEP 1 - GO TO HOME
        self.go_to_pose('pHome')
        ## STEP 2 - GO TO SEARCH ZONE
        self.go_to_pose('pSearch')
        ## SOLVER TAG ISSUE
        self.tag_found = 0
        self.init_pose = 1                                 
        # STEP 3 - KEEP ROTATING
        while not rospy.is_shutdown():
            # IF TAG WAS NOT FOUND, ROTATE
            if self.tag_found is 0:
                self.search_tag()
                rospy.loginfo('############# ROTATION COMPLETE')
                
            # IF TAG WAS FOUND, GO TO TARGET POSITION
            elif self.tag_found is 1:
                rospy.loginfo('############# TAG FOUND')
                self.go_to_pose('pTarget')
                rospy.loginfo('############# FINISHING, GO TO HOME')
                self.go_to_pose('pHome')
                sys.exit()

                   

if __name__ == '__main__':
    try:
        # INITIALIZE YOUR OBJECT
        manip = openManipulatorPRO()
        # INITIALIZE THE OPERATION BY CLICKING ENTER
        raw_input("Press Enter to start!")
        # OPERATION
        manip.process()  

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass       