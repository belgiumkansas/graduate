#!/usr/bin/env python

from ros_pololu_servo.msg import MotorStateList
from ros_pololu_servo.msg import MotorState
from ros_pololu_servo.msg import MotorCommand
from std_msgs.msg import Float64
from std_msgs.msg import Bool



import rospy
import sys, signal

#key stroke interrupt
def signal_handler(signal, frame):
    print("\nprogram exiting gracefully")
    sys.exit(0)

class IR_filter_pub:

    def __init__(self):
        self.IR_side_avg = 0
        self.IR_frnt_avg = 0
        self.SAMPLES = 4


    #republish pulse data as float64
    def callback(self, data):

        #publish data
        state = Float64()
        self.IR_side_avg += data.motor_states[2].pulse/self.SAMPLES
        self.IR_side_avg -= self.IR_side_avg/self.SAMPLES
        state = self.IR_side_avg
        pub_data.publish(state)
        #publish static setpoint
        setpoint = Float64()
        setpoint.data = 82.0
        pub_setpoint.publish(setpoint)
        #publish active PID message
        enable = Bool()
        enable.data = True
        pub_enable.publish(enable)


if __name__ == '__main__':
    print "hello cruel world"
    #initiate keystroke interrupt
    signal.signal(signal.SIGINT, signal_handler)
    IR = IR_filter_pub()


    rospy.init_node("IR_data_pub", anonymous = True)
    pub_data=rospy.Publisher("/state", Float64, queue_size = 1 )
    pub_setpoint=rospy.Publisher("/setpoint", Float64, queue_size = 1)
    pub_enable=rospy.Publisher("/pid_enable", Bool, queue_size = 1)

    rospy.Subscriber("/pololu/motor_states", MotorStateList, IR.callback)

    rospy.spin()

    print "goodbye cruel world my node has died"
