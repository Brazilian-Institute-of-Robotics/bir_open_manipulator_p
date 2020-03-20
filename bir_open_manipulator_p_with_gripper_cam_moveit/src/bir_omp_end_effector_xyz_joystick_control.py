#!/usr/bin/env python
import sys
import rospy
from math import pi
import moveit_commander
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import DisplayTrajectory

# CONSTANTS
MOVEIT_PLANNING_TIME = 5.0
MOVEIT_POSE_TOLERANCE = 0.010
STEP_IN_X = 0.05
STEP_IN_Y = 0.05
STEP_IN_Z = 0.05

class openManipulatorPRO:
    def __init__(self):
        # ROS node Init
        rospy.init_node('OMP_gripper_moveit_commander')              
        # MoveIt Init
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory,queue_size=20)
        # Publisher/Subscriber
        self.joystick_subscriber = rospy.Subscriber('/joy', Joy, self.joystickCallback, queue_size=10)
        self.end_effector_publisher = rospy.Publisher('/end_effector_axes_pose', Pose, queue_size=10, latch=True)
        # MoveIt - Vars for End Effector
        self.ee_name = self.group.get_end_effector_link()
        # MoveIt - Restrictions
        self.group.set_goal_position_tolerance(MOVEIT_POSE_TOLERANCE)
        self.group.set_planning_time(MOVEIT_PLANNING_TIME)
        # Joystick - Vars
        self.command_rt = 0
        self.command_lt = 0
        self.command_x = 0
        self.command_y = 0
        self.command_finish = 0  

    def joystickCallback(self, joyMsg):
        # RT -1 || +1 None
        self.command_rt = joyMsg.axes[4]
        # LT -1 || +1 None
        self.command_lt = joyMsg.axes[5]
        # LEFT  +1 || RIGHT -1
        self.command_x = joyMsg.axes[6]
        # UPPER +1 || DOWN -1
        self.command_y = joyMsg.axes[7]
        # FINISH COMMAND - PRESS A
        self.command_finish = joyMsg.buttons[0]

    def publishEndEffectorPose(self, axis):
        get_ee_pose = self.group.get_current_pose(end_effector_link=self.ee_name)
        pub_ee_pose = get_ee_pose.pose
        # Define values to publish
        pub_ee_pose.orientation.x = axis # Axis that you controlled
        pub_ee_pose.orientation.y = MOVEIT_POSE_TOLERANCE # Moveit Tolerance
        self.end_effector_publisher.publish(pub_ee_pose)
        pub_ee_pose.orientation.x = 99.0                  # For plot decision (command or time)
        self.end_effector_publisher.publish(pub_ee_pose)

    def goToNamedPose(self, pose_name):
        """
        Plan and execute a named pose
        """
        # 1 - PASS YOUR POSE SAVED ON SETUP ASSISTANT
        self.group.set_named_target(pose_name)
        # 2 - PLAN AND EXECUTE
        self.group.go(wait=True)
        # 3 - PREVENT RESIDUAL MOVEMENT
        self.group.stop()
        # 4 - CLEAR TARGET GOAL
        self.group.clear_pose_targets()

    def shiftAxisPoseTargetAndMoveIt(self, axis, value):
        '''
        Get current end effector pose, add value to corresponding axis [0..2: x,y,z], set as pose target and try to execute a motion.
        - **axis** is a value that represent each axis -> [0,1,2] == [x,y,z]
        - **value** is the value to be add in desired axis
        '''
        # Get current end effector pose, change it and set as target pose
        self.group.shift_pose_target(axis=axis, value=value, end_effector_link=self.ee_name)
        
        # Plan and Execute motion - Way 1
        #self.group.go(wait=True)

        # Plan and execute a motion for target pose if possible - Way 2
        plan = self.group.plan()
        if plan.joint_trajectory.points:
            self.group.execute(plan,wait=True)
            # Publish end pose
            self.publishEndEffectorPose(axis=axis)
            # Print
            rospy.loginfo('--- TRAJECTORY SUCCEED.')
        else:
            rospy.logerr('--- FAIL TRAJECTORY, TRY AGAIN!')
        
        # Stop residual movement
        self.group.stop()
        
        # Clear current pose target
        self.group.clear_pose_target(end_effector_link=self.ee_name)

    def endEffectorJoystickCommander(self):
        '''
        Set a pose for end effector with a joystick through MoveIt!
        Commands:

        +X -> Left Directional

        -X -> Right Directional

        +Y -> Upper Directional

        -Y -> Lower Directional

        +Z -> RT

        -Z -> LT
        '''
        # Go to a start pose
        self.goToNamedPose(pose_name='pCatch')
        self.publishEndEffectorPose(axis=0.0)
        rospy.loginfo('Wait for joy commands ...')
        # Commander Loop
        while not rospy.is_shutdown():
            # Z Axis
            if self.command_rt == -1:
                rospy.loginfo('Command +Z')
                self.shiftAxisPoseTargetAndMoveIt(axis=2, value=STEP_IN_Z)
            elif self.command_lt == -1:
                rospy.loginfo('Command -Z')
                self.shiftAxisPoseTargetAndMoveIt(axis=2, value=-STEP_IN_Z)

            # Y Axis
            elif self.command_y == 1:
                rospy.loginfo('Command +Y')
                self.shiftAxisPoseTargetAndMoveIt(axis=1, value=STEP_IN_Y)
            elif self.command_y == -1:
                rospy.loginfo('Command -Y')
                self.shiftAxisPoseTargetAndMoveIt(axis=1, value=-STEP_IN_Y)

            # X Axis
            elif self.command_x == 1:
                rospy.loginfo('Command +X')
                self.shiftAxisPoseTargetAndMoveIt(axis=0, value=STEP_IN_X)
            elif self.command_x == -1:
                rospy.loginfo('Command -X')
                self.shiftAxisPoseTargetAndMoveIt(axis=0, value=-STEP_IN_X)

            # Finish program
            elif self.command_finish == 1:
                sys.exit()

            # Sleeping
            rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        # INITIALIZE YOUR OBJECT
        omanip = openManipulatorPRO()
        # INITIALIZE THE OPERATION BY CLICKING ENTER
        #raw_input("Press Enter to start!")
        # OPERATION
        omanip.endEffectorJoystickCommander()

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass       