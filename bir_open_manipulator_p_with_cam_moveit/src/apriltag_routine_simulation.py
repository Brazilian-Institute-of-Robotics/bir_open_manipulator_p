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

class openManipulator:
    def __init__(self):
        # ROS NODE INIT
        rospy.init_node('apriltag_detector_go2goal')        
        # MOVEIT INIT
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")
        #self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
        #                                                    moveit_msgs.msg.DisplayTrajectory,
        #                                                    queue_size=20)
        self.tag_id_subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray,self.callback)
        # MOVEIT RESTRICTIONS
        self.group.set_goal_position_tolerance(0.1)          # GOAL TOLERANCE
        self.group.set_planning_time(5)                      # TIME TO PLANNING
        # CODE PARAMETERS
        self.rotate_degree = -pi*10/180       
        self.limit_j1 = -pi
        self.position_tolerance = 0.05
        # MOVEIT RESTRICTIONS
        self.group.set_goal_position_tolerance(self.position_tolerance)     # GOAL TOLERANCE
        self.group.set_planning_time(5)                                     # TIME TO PLANNING
        # TAG DETECTION VARS
        self.id_found = 0                                    # FOUND TAG
        self.seq_counts = 0                                  # SEQ COUNTS THAT THE RBG GET THE FRAME
        self.MAX_SEQS = 25                                   # SEQ TO DETECT TAG

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

    # FUNCTION - SET JOINTS ACTUAL - TARGET
    def rotateJ1(self):
        # DEFINE A VARIABLE FORMAT EQUIVALENT TO JOINTS STATES
        joint_goal = self.group.get_current_joint_values()
        # DECISION :: FINISH THE NODE IF JOINT ANGLE IS CLOSE TO PATH END
        if abs(joint_goal[0] - self.limit_j1) < 0.1:
            # 2.1 - BEFORE END NODE, SEND TO pSearch
            rospy.loginfo('############# GO TO PHOME, MAX LIMIT IN SEARCH')
            self.go_to_pose('pHome')
            sys.exit()

        # MOVE MANIPULATOR BASED ON SENSE PARAMETER
        joint_goal[0] = joint_goal[0] + self.rotate_degree
        # 5 - GO!
        self.group.go(joints=joint_goal, wait=True)
        # 6 - Print Joint state
        rospy.loginfo('J1 '+str(round(joint_goal[0]*180/pi,1))+' degrees'+' | '+ str(joint_goal[0])+' rad')
        # 7 - STOP  ANY RESIDUAL MOVEMENT
        self.group.stop()
        # 8 - CLEAR TARGET GOAL
        self.group.clear_pose_targets()

    # FUNCTION - CALLBACK
    def callback(self,data):
        if (self.id_found is not 1) and (len(data.detections) is not 0):
            if data.detections[0].id[0] == 1:
                self.seq_counts = self.seq_counts + 1
            else:
                self.seq_counts = 0
        
            if self.seq_counts == self.MAX_SEQS:
                self.id_found = 1
        else:
            pass


    # FUNCTION - PROCESS ROUTINE
    def process(self):
        ## STEP 1 - GO TO HOME
        self.go_to_pose('pHome')
        ## STEP 2 - GO TO SEARCH ZONE
        self.go_to_pose('pSearch')
        ## SOLVER TAG ISSUE
        self.id_found = 0                                 
        # STEP 3 - KEEP ROTATING
        while not rospy.is_shutdown():
            # IF TAG WAS NOT FOUND, ROTATE
            if self.id_found is 0:
                self.rotateJ1()
                rospy.loginfo('############# ROTATION COMPLETE')
                
            # IF TAG WAS FOUND, GO TO TARGET POSITION
            elif self.id_found is 1:
                rospy.loginfo('############# TAG FOUND')
                self.go_to_pose('pTarget')
                rospy.loginfo('############# FINISHING, GO TO HOME')
                self.go_to_pose('pHome')
                sys.exit()

                   

if __name__ == '__main__':
    try:
        # INITIALIZE YOUR OBJECT
        manip = openManipulator()
        # INITIALIZE THE OPERATION BY CLICKING ENTER
        raw_input("Press Enter to start!")
        # OPERATION
        manip.process()  

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass       