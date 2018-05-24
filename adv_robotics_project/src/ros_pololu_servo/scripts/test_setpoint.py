#!/usr/bin/env python

import rospy
from rospy import init_node, is_shutdown

from std_msgs.msg import Float64
from std_msgs.msg import Bool
import sys, signal


def signal_handler(signal, frame):
    print("\nprogram exiting gracefully")
    sys.exit(0)

if __name__ == '__main__':

    signal.signal(signal.SIGINT, signal_handler)

    init_node('testing_temporary')
    pub_motor_set = rospy.Publisher("/motor_setpoint", Float64, queue_size = 1)
    pid_enable = rospy.Publisher("/pid_enable", Bool, queue_size = 1)

    setpoint = Float64()
    setpoint = 60

    wait = rospy.Rate(20)

    while not rospy.is_shutdown():
        pub_motor_set.publish(40)
        pid_enable.publish(True)
        wait.sleep()
