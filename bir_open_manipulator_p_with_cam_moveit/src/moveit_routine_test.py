#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from apriltag_ros.msg import AprilTagDetectionArray
from moveit_commander.conversions import pose_to_list

class openManipulator:
    def __init__(self):
        # ROS NODE INIT
        rospy.init_node('moveit_routine_fix', anonymous=True)
        self.stopper = Float64()
        self.stopper.data = 0        
        # MOVEIT INIT
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "arm"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)
        # DEFINE PARAMETERS
        self.group.set_planning_time(5)                             # TIME TO PLANNING

    def security_sequential_execution(self, pose_name):
        # SET TARGET & GO
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

    def security_sequential_execution2(self, pose_name):
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
            for i in range(0, len(jointsActual)):
                if abs(abs(jointsActual[i])-abs(jointsTarget[i])) <= 0.1:
                    stopCondition += 1
            # STOP CONDITION
            if stopCondition == len(jointsActual):
                self.group.stop()
                self.group.clear_pose_targets()
                rospy.sleep(1)
                break
                   
if __name__ == '__main__':
    try:
        manip = openManipulator()
        raw_input("Press Enter to start!")
        manip.security_sequential_execution2('pHome')
        manip.security_sequential_execution2('pSearch')

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass      