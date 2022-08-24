#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Imu
import numpy as np
import math
import time
import torch
from torch import device, nn
from numpy import *

class ShallowRegressionLSTM(nn.Module):
    def __init__(self, num_sensors, hidden_units):
        super().__init__()
        self.num_sensors = num_sensors  # this is the number of features
        self.hidden_units = hidden_units
        self.num_layers = 1

        self.lstm = nn.LSTM(
            input_size=num_sensors,
            hidden_size=hidden_units,
            batch_first=True,
            num_layers=self.num_layers
        )

        self.linear = nn.Linear(in_features=self.hidden_units, out_features=1)

    def forward(self, x):
        batch_size = x.shape[0]
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        h0 = torch.zeros(self.num_layers, batch_size, self.hidden_units).requires_grad_().to(device)
        c0 = torch.zeros(self.num_layers, batch_size, self.hidden_units).requires_grad_().to(device)
        
        _, (hn, cn) = self.lstm(x, (h0, c0))
        out = self.linear(hn[0]).flatten()  # First dim of Hn is num_layers, which is set to 1 above.

        return out, hn, cn







def foot_callback(data,imu_array):
    #use mean and std to z score the data
    g = 9.81
    data_mean = [-0.949, 0.157289, 0.882190, 0.149684, 0.061574, 0.038574, -1.117815, 0.035271, -0.331293, 0.008410, 0.009577, 0.083401]
    stdev = [1.145217, 0.707444, 1.018298, 1.409675, 3.821612, 1.637908, 0.536504, 0.373013, 0.725947, 1.494172, 2.900099, 0.845181]
    imu_array[0,0] = ((data.linear_acceleration.x/g)-data_mean[0])/stdev[0]
    imu_array[0,1] = ((data.linear_acceleration.y/g)-data_mean[1])/stdev[1]
    imu_array[0,2] = ((data.linear_acceleration.z/g)-data_mean[2])/stdev[2]

    imu_array[0,3] = (data.angular_velocity.x-data_mean[3])/stdev[3]
    imu_array[0,4] = (data.angular_velocity.y-data_mean[4])/stdev[4]
    imu_array[0,5] = (data.angular_velocity.z-data_mean[5])/stdev[5]

def shank_callback(data,imu_array):
    #use mean and std to z score the data
    g = 9.81
    data_mean = [-0.949, 0.157289, 0.882190, 0.149684, 0.061574, 0.038574, -1.117815, 0.035271, -0.331293, 0.008410, 0.009577, 0.083401]
    stdev = [1.145217, 0.707444, 1.018298, 1.409675, 3.821612, 1.637908, 0.536504, 0.373013, 0.725947, 1.494172, 2.900099, 0.845181]
    imu_array[0,6] = ((data.linear_acceleration.x/g)-data_mean[6])/stdev[6]
    imu_array[0,7] = ((data.linear_acceleration.y/g)-data_mean[7])/stdev[7]
    imu_array[0,8] = ((data.linear_acceleration.z/g)-data_mean[8])/stdev[8]

    imu_array[0,9] = (data.angular_velocity.x-data_mean[9])/stdev[9]
    imu_array[0,10] = (data.angular_velocity.y-data_mean[10])/stdev[10]
    imu_array[0,11] = (data.angular_velocity.z-data_mean[11])/stdev[11]

def listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    process_rate = 100
    
    speed = Float32()
    rospy.init_node('infer_speed', anonymous=True)
    pub = rospy.Publisher('speed',Float32, queue_size=10)
    
    imu_array = np.zeros((1,12))
    
    rospy.Subscriber("/footSensor/imu/data", Imu, foot_callback, imu_array)
    rospy.Subscriber("/shankSensor/imu/data", Imu, shank_callback, imu_array)
    rate = rospy.Rate(process_rate)

    #used to z-score the data
    
    target_mean = 1.1710513157512659
    target_stdev = 0.43418490501484597

    temporal_data = torch.zeros((imu_array.shape[0],imu_array.shape[1])).float()
    num_features = imu_array.shape[1]


    

    #Specify the model here
    #Load in the model


    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print('run with',device)


    # num_hidden_units = 4
    num_hidden_units = 12
    PATH_TO_PT_MODEL = '/home/rover/v_estimate_data/model_12_hidden_units_v3.pt'
    model = ShallowRegressionLSTM(num_sensors=num_features, hidden_units=num_hidden_units)
    model.load_state_dict(torch.load(PATH_TO_PT_MODEL))

    model.eval()
    model.to(device)

    #How long the sequence of data is 
    sequence_length = 60
    
    start_time = time.time()

    with torch.no_grad():
        while not rospy.is_shutdown():

            curr_data = imu_array
            #pub_imu.publish(Float32MultiArray(curr_data))
            if temporal_data.shape[0] < sequence_length:
                temporal_data = torch.cat((temporal_data,torch.reshape(torch.tensor(curr_data).float(),(1,num_features))))

            else:
                temporal_data = temporal_data[1:]
                temporal_data = torch.cat((temporal_data,torch.reshape(torch.tensor(curr_data).float(),(1,num_features))))

                input = torch.reshape(temporal_data,(1,temporal_data.shape[0], temporal_data.shape[1])) #Reshape input so batch size is 1

                pred ,hn,cn = model(input.to(device))
                
                #LSTM_v2 does not z-score the targets
                # speed_pred = pred*target_stdev  + target_mean
                speed_pred = pred.to('cpu')
                
                pub.publish(speed_pred[0].numpy())
                rate.sleep()

if __name__ == '__main__':
    listener()