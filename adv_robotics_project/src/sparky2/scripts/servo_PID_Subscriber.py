#!/usr/bin/env python

from ros_pololu_servo.msg import MotorStateList
from ros_pololu_servo.msg import MotorState
from ros_pololu_servo.msg import MotorCommand
from std_msgs.msg import Float64

import rospy
import sys, signal

def callback(data):

    servo_cmd = MotorCommand()
    servo_cmd.joint_name = 'Servo'
    servo_cmd.position = (data.data * .007)
    pub_MotorCmd.publish(servo_cmd)


if __name__ == '__main__':

    print "hello I am the servo controller"

    rospy.init_node("servo_control_pub", anonymous = False)
    pub_MotorCmd = rospy.Publisher("/pololu/command", MotorCommand, queue_size = 1)
    rospy.Subscriber("/servo_control_effort", Float64, callback)

    rospy.spin()
