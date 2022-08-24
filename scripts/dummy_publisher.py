#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
import numpy as np
import math
import time
from numpy import *
import random


def talker():

    process_rate = 100
    rospy.init_node('dummy_speed', anonymous=True)
    rate = rospy.Rate(process_rate)
    speed = Float32()
    
    pub = rospy.Publisher('speed',Float32, queue_size=10)
    while not rospy.is_shutdown():
        speed = 1 + random.random()*[-1,1][random.randrange(2)]
        pub.publish(speed)
        rate.sleep()

if __name__ == '__main__':
    talker()
