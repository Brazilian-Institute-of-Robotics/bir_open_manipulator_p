#!/usr/bin/env python
import rospy
import thread
from std_msgs.msg import Float64
from open_manipulator_msgs.srv import SetActuatorState

# ROS INIT
rospy.init_node('STOP_OMP_ROBOT')
# START YOUR SERVICE
rospy.wait_for_service('/open_manipulator_pro/set_actuator_state') 
serviceStop = rospy.ServiceProxy('/open_manipulator_pro/set_actuator_state',SetActuatorState)
# LOOP
while not rospy.is_shutdown():
    STOP_VALUE = int(raw_input('> 1 STOP | 99 FINISH NODE: '))
    if STOP_VALUE == 1:
        serviceStop(False)
        serviceStop(True)
    elif STOP_VALUE == 99:
        break