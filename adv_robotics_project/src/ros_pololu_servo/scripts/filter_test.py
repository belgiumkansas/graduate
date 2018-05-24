#!/usr/bin/env python

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3


#low pass magic
from numpy import cos, sin, pi, absolute, arange
from scipy.signal import kaiserord, lfilter, firwin, freqz

import scipy
import rospy
import sys, signal

def signal_handler(signal, frame):
    print("\nprogram exiting gracefully")
    sys.exit(0)


class filter_test:

    def __init__(self, sample_rate):

        nyq_rate = sample_rate/2
        width = 5.0/nyq_rate
        ripple_db = 60.0
        N, beta = kaiserord(ripple_db, width)
        cutoff_hz = 10.0
        self.taps = firwin(N, cutoff_hz/nyq_rate, window=('kaiser', beta))
        self.buffer = [0]*len(self.taps)

        print str(len(self.taps)) +" "+ str(len(self.buffer))


    def callback(self, data):
        #print "holla your the calla"


        y_data = data.linear_acceleration.y
        self.buffer.pop()
        self.buffer.insert(0,y_data)
        #print self.buffer

        filtered_y = lfilter(self.taps, 1.0, self.buffer)

        print filtered_y[90]

        pub_unfiltered.publish(y_data)
        pub_filtered.publish(filtered_y[90])


if __name__ == '__main__':
    print "hello filter test"

    signal.signal(signal.SIGINT, signal_handler)

    filter_and_pub = filter_test(125)

    rospy.init_node("filter_test", anonymous = True)

    pub_unfiltered = rospy.Publisher("/dirty", Float64, queue_size = 1)
    pub_filtered = rospy.Publisher("/clean", Float64, queue_size = 1)

    rospy.Subscriber("/imu/data_raw", Imu, filter_and_pub.callback)

    rospy.spin()

    print "Spinning till murdered"
