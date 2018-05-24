#!/usr/bin/env python

from ros_pololu_servo.msg import MotorStateList
from ros_pololu_servo.msg import MotorState
from ros_pololu_servo.msg import MotorCommand
from std_msgs.msg import Float64
from std_msgs.msg import Bool
import numpy
import rospy
import sys, signal


#key stroke interrupt
def signal_handler(signal, frame):
    print("\nprogram exiting gracefully")
    sys.exit(0)

class state_pub:

    def __init__(self):
        self.test = 0

    def callback(self, data):

        '''parse messages and publish'''
        for i in range(0, len(data.motor_states)):
            if(data.motor_states[i].name == "IR-L"):
                state_l = Float64()
                state_l = data.motor_states[i].pulse
                IR_left_pub.publish(state_l)

            elif(data.motor_states[i].name == "IR-R"):
                state_r = Float64()
                state_r = data.motor_states[i].pulse
                IR_front_pub.publish(state_r)

            elif(data.motor_states[i].name == "BEMF"):
                bemf = Float64
                bemf = data.motor_states[i].pulse
                BEMF_pub.publish(bemf)


            elif(data.motor_states[i].name == "trigger"):
                trigger = Float64
                trigger = data.motor_states[i].pulse
                trigger_pub.publish(trigger)

            elif(data.motor_states[i].name == "ESC"):
                esc_data = Float64
                esc_data = data.motor_states[i].radians
                esc_pub.publish(esc_data)

            elif(data.motor_states[i].name == "Servo"):
                servo_data = Float64
                servo_data = data.motor_states[i].radians
                servo_pub.publish(servo_data)

        '''only publish when mosfet is closed'''
        if(trigger >= 20):
            BEMF_clean_pub.publish(bemf)



if __name__=='__main__':
    print "what is my purpose?"

    signal.signal(signal.SIGINT, signal_handler)

    pass_butter = state_pub()

    rospy.init_node("maestro_publisher", anonymous = False)

    IR_left_pub = rospy.Publisher("/maestro/left_IR", Float64,queue_size = 1)
    IR_front_pub = rospy.Publisher("/maestro/front_IR", Float64, queue_size = 1)
    BEMF_pub = rospy.Publisher("/maestro/BEMF", Float64, queue_size = 1)
    trigger_pub = rospy.Publisher("/maestro/trigger", Float64, queue_size = 1)
    esc_pub = rospy.Publisher("/maestro/esc_state", Float64, queue_size = 1)
    servo_pub = rospy.Publisher("/maestro/servo_state", Float64, queue_size = 1)
    BEMF_clean_pub = rospy.Publisher("/maestro/BEMF_clean", Float64, queue_size = 1)

    rospy.Subscriber("/pololu/motor_states", MotorStateList, pass_butter.callback)

    print "too pass the butter"

    rospy.spin()
