#!/usr/bin/env python
import rospy
from rospy import init_node, is_shutdown
from ros_pololu_servo.msg import MotorCommand
import sys, signal


def signal_handler(signal, frame):
    print("\nprogram exiting gracefully")
    sys.exit(0)

if __name__ == '__main__':

    signal.signal(signal.SIGINT, signal_handler)

    init_node('forward_node')
    pub_esc = rospy.Publisher("/pololu/command", MotorCommand, queue_size = 1)

    speed_cmd = MotorCommand()
    speed_cmd.joint_name = "ESC"
    current_speed = .33

    while not is_shutdown():
        print "faster or slower"
        command = raw_input()
        print command + "received"

        if(command == "w"):
            current_speed += .02

        elif(command == "s"):
            current_speed -= .02
        else:
            print "shit command"

        print "current speed: " + str(current_speed)
        speed_cmd.position = current_speed

        pub_esc.publish(speed_cmd)
