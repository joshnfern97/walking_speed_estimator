#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
import numpy as np
import math
import time
from numpy import *
import random
import csv
from datetime import datetime


def speed_callback(data,container):
    container[13] = data.data

def filtered_speed_callback(data,container):
    container[14] = data.data

def heading_callback(data,container):
    container[15] = data.data

def lng_callback(data,container):
    container[16] = data.data

def lat_callback(data,container):
    container[17] = data.data

def foot_callback(data,container):
    container[1] = data.linear_acceleration.x
    container[2] = data.linear_acceleration.y
    container[3] = data.linear_acceleration.z

    container[4] = data.angular_velocity.x
    container[5] = data.angular_velocity.y
    container[6] = data.angular_velocity.z


def shank_callback(data, container):
    container[7] = data.linear_acceleration.x
    container[8] = data.linear_acceleration.y
    container[9] = data.linear_acceleration.z

    container[10] = data.angular_velocity.x
    container[11] = data.angular_velocity.y
    container[12] = data.angular_velocity.z

def listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.init_node('listener', anonymous=True)
    data_log_titles = ['t','f_ax','f_ay','f_az','f_wx','f_wy','f_wz','s_ax','s_ay','s_az','s_wx','s_wy','s_wz', 'speed', 'filtered_speed','heading','longitude','latitude']
    log_length = len(data_log_titles)
    container = np.zeros(log_length)
    process_rate = 100
    rate = rospy.Rate(process_rate)
    filtered_speed = 0
    
    now = datetime.now()
    dt_string = now.strftime('%d-%m-%Y-%H-%M-%S')
    file_path = '/home/rover/v_estimate_data/csv_files/walking_time_'+ dt_string +'.csv'
    rospy.Subscriber("/speed", Float32, speed_callback, container)
    rospy.Subscriber("/filtered_speed", Float32, filtered_speed_callback, container)
    rospy.Subscriber("/footSensor/imu/data", Imu, foot_callback, container)
    rospy.Subscriber("/shankSensor/imu/data", Imu, shank_callback, container)
    rospy.Subscriber("/heading", Float32, heading_callback, container)
    rospy.Subscriber("/latitude", Float32, lat_callback, container)
    rospy.Subscriber("/longitude", Float32, lng_callback, container)
    
    start_time = time.time()
    with open(file_path,'w',encoding='UTF8') as f:
        writer = csv.writer(f)
        writer.writerow(data_log_titles)


    
        while not rospy.is_shutdown():
                curr_time = time.time()-start_time

                container[0] = curr_time
                
                writer.writerow(container)



                rate.sleep()

if __name__ == '__main__':
    listener()

