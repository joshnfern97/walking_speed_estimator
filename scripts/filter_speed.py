#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
import numpy as np
import math
import time
from numpy import *





def filter_callback(data, args):
    prior_speed = args[0]
    container = args[1]
    average_window = 100

    prior_speed.append(data.data)
    #print(len(prior_speed))
    if len(prior_speed) > average_window:
        del prior_speed[0]

    container[0] = np.mean(prior_speed)
    


def listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.init_node('filter_speed', anonymous=True)

    process_rate = 100
    rate = rospy.Rate(process_rate)
    filtered_speed = 0
    container = [filtered_speed]
    pub = rospy.Publisher('filtered_speed',Float32, queue_size=10)
    prior_speed = []
    rospy.Subscriber("/speed", Float32, filter_callback, (prior_speed,container))
    
    while not rospy.is_shutdown():

            filtered_speed = container[0]
            pub.publish(filtered_speed)
            rate.sleep()

if __name__ == '__main__':
    listener()