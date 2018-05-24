#!/usr/bin/env python

from ros_pololu_servo.msg import MotorCommand

from std_msgs.msg import Float64
from rospy import init_node, is_shutdown

import rospy
import sys, signal

def signal_handler(signal, frame):
    print("\nprogram exiting gracefully")
    sys.exit(0)


class esc_controller:
    def __init__(self):
        self.set_state = 20 #bemf
        self.stored_state = 0 #bemf data

    def callback_set_state(self, data):
        print "set_state_callback"
        current_bemf = data.data #motor angle

        speed_cmd = MotorCommand()
        speed_cmd.joint_name = "ESC"

        if(current_bemf <= self.set_state):
            motor_angle = .35

        elif(current_bemf > self.set_state):
            motor_angle = .2

        speed_cmd.position = motor_angle
        pub_esc.publish(speed_cmd)





if __name__ == '__main__':
    print "hello back emf"
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('esc_controller')

    bad_esc = esc_controller()
    pub_esc = rospy.Publisher("/pololu/command", MotorCommand, queue_size = 1)


    rospy.Subscriber("/BEMF_clean", Float64, bad_esc.callback_set_state)



    while not is_shutdown():
        print "faster or slower"
        command = raw_input()

        print command + "received"

        if(command == "w"):
            bad_esc.set_state += 1

        elif(command == "s"):
            bad_esc.set_state -= 1
        else:
            print "shit command"

        print "current set point: " + str(bad_esc.set_state)
