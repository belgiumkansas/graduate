#!/usr/bin/env python

from ros_pololu_servo.msg import MotorStateList
from ros_pololu_servo.msg import MotorState
from ros_pololu_servo.msg import MotorCommand
from std_msgs.msg import Float64


import rospy
import sys, signal


def callback(data):
    control_effort = data.data

    esc_cmd = MotorCommand()

    if(control_effort < 0):
        esc_cmd.position = .305
    elif(control_effort >= 60):
        esc_cmd.position = .345
    else:
        esc_cmd.position = .305 + control_effort/1200.0

    esc_cmd.joint_name = 'ESC'
    pub_MotorCmd.publish(esc_cmd)

    '''comment me out to return steering controll'''
    servo_cmd = MotorCommand()
    servo_cmd.joint_name = 'Servo'
    servo_cmd.position = 0.0
    pub_MotorCmd.publish(servo_cmd)



if __name__ == '__main__':

    print "hello I am the esc and servo. for now"

    rospy.init_node("motor_control_pub", anonymous = False)
    pub_MotorCmd = rospy.Publisher("/pololu/command", MotorCommand, queue_size = 1)


    rospy.Subscriber("/motor_control_effort", Float64, callback)

    rospy.spin()
