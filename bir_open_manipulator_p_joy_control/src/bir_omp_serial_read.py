#!/usr/bin/env python
import rospy
import serial                                                           # Serial communication
import numpy as np
from std_msgs.msg import Float64

def serial_reader(publisher):
    while not rospy.is_shutdown():
        pass

# MAIN
if __name__ == '__main__':
    try:
        rospy.init_node('bir_bci_serial_reader')
        pub = rospy.Publisher('/bci_serial_reader', Float64,latch=False, queue_size=10)
        serial_reader(publisher=pub)

    except rospy.ROSInterruptException:
        pass

