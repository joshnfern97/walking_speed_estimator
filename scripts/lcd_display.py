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
# Import LCD library
from RPLCD import i2c

# Import sleep library
from time import sleep

def filtered_speed_callback(data,container):
    container[3] = data.data

def heading_callback(data,container):
    container[2] = data.data

def lng_callback(data,container):
    container[0] = data.data

def lat_callback(data,container):
    container[1] = data.data


def listener():
    

    try:
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.

        rospy.init_node('lcd_disp', anonymous=True)
        

        # constants to initialise the LCD
        lcdmode = 'i2c'
        cols = 16
        rows = 2
        charmap = 'A00'
        i2c_expander = 'PCF8574'

        # Generally 27 is the address;Find yours using: i2cdetect -y 1 
        address = 0x27 
        port = 8 # 0 on an older Raspberry Pi

        # Initialise the LCD
        lcd = i2c.CharLCD(i2c_expander, address, port=port, charmap=charmap,
            cols=cols, rows=rows)


        container = np.zeros(4)
        process_rate = 10
        rate = rospy.Rate(process_rate)
        filtered_speed = 0
        

        rospy.Subscriber("/filtered_speed", Float32, filtered_speed_callback, container)
        rospy.Subscriber("/heading", Float32, heading_callback, container)
        rospy.Subscriber("/latitude", Float32, lat_callback, container)
        rospy.Subscriber("/longitude", Float32, lng_callback, container)
        

        
        while not rospy.is_shutdown():
        
            curr_lng = container[0]
            curr_lat = container[1]
            curr_heading = container[2]
            curr_speed = container[3]
            
            if curr_lat==0:
                lcd.write_string('No GPS')
            else:
                lcd.write_string('GPS Lock')
            lcd.crlf()
            second_line = "{:.1f}".format(curr_heading)+" | " + "{:.2f}".format(curr_speed) + "m/s"
            lcd.write_string(second_line)
            lcd.crlf()



            rate.sleep()
    except:
        pass
        lcd.close(clear=True)
        # Switch off backlight
        lcd.backlight_enabled = False 
if __name__ == '__main__':
    
    listener()
    
