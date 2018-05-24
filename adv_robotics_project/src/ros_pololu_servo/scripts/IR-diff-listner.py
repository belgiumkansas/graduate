#!/usr/bin/env python

from ros_pololu_servo.msg import MotorStateList
from ros_pololu_servo.msg import MotorState
from ros_pololu_servo.msg import MotorCommand

import rospy
import sys, signal

names=['IR-L', 'IR-R', 'Servo']

def signal_handler(signal, frame):
    print("\nprogram exiting gracefully")
    sys.exit(0)


def callback(data):

    diff =  data.motor_states[1].pulse - data.motor_states[2].pulse
    print diff

    servo_cmd = MotorCommand()
    if(diff < -100 ):
        servo_cmd.position = float(-0.6)
    elif(diff > 100):
        servo_cmd.position = 0.6
    else:
        servo_cmd.position = diff/167.0

    servo_cmd.position = - servo_cmd.position
    servo_cmd.joint_name = "Servo"
    pub.publish(servo_cmd)



if __name__ == '__main__':

    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('IR_pid_listener', anonymous = True)
    pub=rospy.Publisher("/pololu/command", MotorCommand, queue_size=4)


    rate = rospy.Rate(10)
    while True:
        rospy.Subscriber("/pololu/motor_states", MotorStateList, callback)
        rate.sleep()
