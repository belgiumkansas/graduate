#!/usr/bin/env python

from ros_pololu_servo.msg import MotorStateList
from ros_pololu_servo.msg import MotorState
from ros_pololu_servo.msg import MotorCommand
from std_msgs.msg import Float64


import rospy
import sys, signal


def callback(data):
    control_effort = data.data

    servo_cmd = MotorCommand()

    if(control_effort < -20):
        servo_cmd.position = float(-.6)
    elif(control_effort > 20):
        servo_cmd.position = float(.6)
    else:
        servo_cmd.position = control_effort/33.0

    servo_cmd.joint_name = "Servo"
    pub_servo.publish(servo_cmd)



if __name__ == '__main__':

    print "hello I am the servo"

    rospy.init_node("Servo_controll", anonymous = True)
    pub_servo = rospy.Publisher("/pololu/command", MotorCommand, queue_size = 1)


    rospy.Subscriber("/control_effort", Float64, callback)

    rospy.spin()
