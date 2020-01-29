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
        rospy.init_node('apriltag_detector_go2goal', anonymous=True)        
        # MOVEIT INIT
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "arm"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)
        self.tag_id_subscriber = rospy.Subscriber('/tag_detections',
                                                  AprilTagDetectionArray,self.callback)
        # DEFINE PARAMETERS
        self.group.set_goal_position_tolerance(0.1)          # GOAL TOLERANCE
        self.group.set_planning_time(5)                      # TIME TO PLANNING
        # AUX VARS
        self.id_found = 0                                    # FOUND TAG
        self.seq_counts = 0                                  # SEQ COUNTS THAT THE RBG GET THE FRAME
        self.rotate_degree = pi*10/180                       # ROTATE 10 DEGREES


    # FUNCTION - GO TO SPECIFIC POSE
    def go_to_pose(self, pose_name):
        ## WE CAN PLAN AND EXECUTE A MOTION FOR THIS GROUP TO A DESIRED SAVED POSE FOR THE END-EFF
        # 1 - PASS YOUR POSE SAVED ON SETUP ASSISTANT
        self.group.set_named_target(pose_name)
        # 2 - PLAN AND EXECUTE
        self.group.go()
        # 3 - PREVENT RESIDUAL MOVEMENT
        self.group.stop()
        # 4 - CLEAR TARGET GOAL
        self.group.clear_pose_targets()

    # FUNCTION - SET JOINTS
    def rotate_joint1(self,sense):
        # 1 - DEFINE A VARIABLE FORMAT EQUIVALENT TO JOINTS
        joint_goal = self.group.get_current_joint_values()
        # 2 - DECISION :: FINISH THE NODE IF JOINT ANGLE IS CLOSE TO PATH END (-90 DEGREES)
        if abs(abs(joint_goal[0]) - abs(-pi/2)) < 0.1:
            # 2.1 - BEFORE END NODE, SEND TO pSearch
            rospy.loginfo('############# GO TO SEARCH POSE, INIT SLEEP MODE')
            self.go_to_pose('pSearch')
            rospy.sleep(8)
            rospy.loginfo('############# GO TO SLEEP')
            self.go_to_pose('pSleep')
            rospy.sleep(8)
            sys.exit()

        # 3 - MOVE MANIPULATOR BASED ON SENSE PARAMETER :: ClockWise
        if sense == 'cw':
            # 3.1 - INSERT DESIRED ANGLE
            joint_goal[0] = joint_goal[0] - self.rotate_degree
            # 3.2 - ANALYZE IF THE DESIRED ANGLE IS HIGHER THAN THE PATH END (-90 DEGREES)
            if abs(joint_goal[0]) > abs(-pi/2):
                joint_goal[0] = -pi/2
        # 4 - MOVE MANIPULATOR BASED ON SENSE PARAMETER :: Anti ClockWise     
        else:
            pass
        # 5 - GO!
        self.group.go(joints=joint_goal, wait=True)
        # 6 - Print Joint state
        rospy.loginfo('joint0 '+str(round(joint_goal[0]*180/pi,1))+' degrees'+' | '+ str(joint_goal[0])+' rad')
        # 7 - STOP  ANY RESIDUAL MOVEMENT
        self.group.stop()
        # 8 - CLEAR TARGET GOAL
        self.group.clear_pose_targets()

    # FUNCTION - CALLBACK
    def callback(self,data):
        if self.id_found is not 1:
            if len(data.detections) is not 0:
                if data.detections[0].id[0] == 1:
                    self.seq_counts = self.seq_counts + 1
                else:
                    self.seq_counts = 0
        
                if self.seq_counts == 100:
                    self.id_found = 1
                    print 1
        else:
            pass


    # FUNCTION - PROCESS ROUTINE
    def process(self):
        ## STEP 1 - GO TO HOME
        self.go_to_pose('pHome')
        rospy.sleep(8)
        ## STEP 2 - GO TO SEARCH ZONE
        self.go_to_pose('pSearch')
        rospy.sleep(8)
        ## SOLVER TAG ISSUE
        self.id_found = 0                                 
        # STEP 3 - KEEP ROTATING
        while not rospy.is_shutdown():
            # PRINT TAG ID
            # 3.1 - IF TAG WAS NOT FOUND, ROTATE
            if self.id_found is 0:
                self.rotate_joint1(sense='cw')
                rospy.loginfo('############# ROTATION COMPLETE')
                rospy.sleep(8)
                
            # 3.2 - IF TAG WAS FOUND, GO TO TARGET POSITION
            if self.id_found is 1:
                rospy.loginfo('############# TAG FOUND')
                self.go_to_pose('pTarget')
                rospy.sleep(8)
                rospy.loginfo('############# RESTART ROUTINE')
                # GO TO pSEARCH
                self.go_to_pose('pSearch')
                rospy.sleep(8)  
                # RESET VARS
                self.id_found = 0
                self.seq_counts = 0
                   

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