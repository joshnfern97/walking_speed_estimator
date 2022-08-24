#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
import numpy as np
import math
import time
from numpy import *
import random

import pandas as pd


def talker():


    df_test = pd.read_csv("/home/rover/v_estimate_data/csv_files/data_pilot-8-8/pilot_2_data_set_test.csv")
    
    process_rate = 100
    rospy.init_node('old_data', anonymous=True)
    rate = rospy.Rate(process_rate)
    foot_imu = Imu()
    shank_imu = Imu()
    
    pub_foot = rospy.Publisher("/footSensor/imu/data",Imu, queue_size=10)
    pub_shank = rospy.Publisher("/shankSensor/imu/data",Imu, queue_size=10)
    i = 0
    while not rospy.is_shutdown():
        curr_data = df_test.iloc[i] #Get Data from current time
        
        foot_imu.linear_acceleration.x = curr_data.f_ax
        foot_imu.linear_acceleration.y = curr_data.f_ay
        foot_imu.linear_acceleration.z = curr_data.f_az
        foot_imu.angular_velocity.x = curr_data.f_wx
        foot_imu.angular_velocity.y = curr_data.f_wy
        foot_imu.angular_velocity.z = curr_data.f_wz
                
        shank_imu.linear_acceleration.x = curr_data.s_ax
        shank_imu.linear_acceleration.y = curr_data.s_ay
        shank_imu.linear_acceleration.z = curr_data.s_az
        shank_imu.angular_velocity.x = curr_data.s_wx
        shank_imu.angular_velocity.y = curr_data.s_wy
        shank_imu.angular_velocity.z = curr_data.s_wz

        pub_foot.publish(foot_imu)
        pub_shank.publish(shank_imu)
        i = i + 1
        rate.sleep()

if __name__ == '__main__':
    talker()