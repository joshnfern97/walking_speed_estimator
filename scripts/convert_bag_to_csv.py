#!/usr/bin/env python3
import rosbag
from std_msgs.msg import Float32
import pandas as pd
import os
import numpy as np

# The bag file should be in the same directory as your terminal
bag_file_name = 'walking_trial_2022-08-05-13-13-34'
file_path = '/home/rover/catkin_ws/src/v_estimate/bag_files/'+bag_file_name+'.bag'
bag = rosbag.Bag(file_path)
#topic = 'cd /your_topic'
column_names = ['x', 'y']
df = pd.DataFrame(columns=column_names)
speed = []
speed_t = []
filtered_speed = []
filtered_speed_t = []
foot_imu_data = np.zeros((1,7))
shank_imu_data = np.zeros((1,7))
# for x in bag.read_messages(topics = '/speed'):
#     print(x)

i = 0
for topic, msg, t in bag.read_messages(topics = "/footSensor/nav/filtered_imu/data"):
    time = msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000
 
    foot_data = np.array([time, msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z,msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
    foot_imu_data = np.vstack((foot_imu_data, foot_data))

for topic, msg, t in bag.read_messages(topics = "/shankSensor/nav/filtered_imu/data"):
    time = msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000
 
    shank_data = np.array([time, msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z,msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
    shank_imu_data = np.vstack((foot_imu_data, foot_data))





for topic, msg, t in bag.read_messages(topics = '/filtered_speed'):
    filtered_speed_t.append(t)
    filtered_speed.append(msg.data)
print(len(filtered_speed))


for topic, msg, t in bag.read_messages(topics = '/speed'):
    speed_t.append(t)
    speed.append(msg.data)


df_speed = pd.DataFrame(list(zip(speed_t, speed)), columns = ['t','data'])


df_filtered_speed = pd.DataFrame(list(zip(filtered_speed_t, filtered_speed)), columns = ['t','data'])

df_foot_imu = pd.DataFrame(foot_imu_data, columns = ['t','ax','ay','az','wx','wy','wz'])
df_shank_imu = pd.DataFrame(shank_imu_data, columns = ['t','ax','ay','az','wx','wy','wz'])

directory = bag_file_name
path = '/home/rover/v_estimate_data/csv_files/'+directory
try:
    os.makedirs(path, exist_ok = True)
    print("Directory '%s' created successfully" %directory)
except OSError as error:
    print("Directory '%s' can not be created")

df_speed.to_csv(path+'/speed.csv')
df_filtered_speed.to_csv(path+'/filtered_speed.csv')
df_foot_imu.to_csv(path+'/foot_imu_data.csv')
df_shank_imu.to_csv(path+'/shank_imu_data.csv')

#df.to_csv('out.csv')