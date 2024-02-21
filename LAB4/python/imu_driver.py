#Code for IMU

#!/usr/bin/python3

import rospy
import serial
from std_msgs.msg import Float64, Header
from std_msgs.msg import String
from LAB4.msg import imu_msg
import sys
import numpy as np
from sensor_msgs.msg import Imu as IMU
from sensor_msgs.msg import MagneticField as MagField 
import math


rospy.init_node("imu_data")
SENSOR_NAME = "imu"
serial_port = sys.argv[1]
serial_baud = 115200


pub = rospy.Publisher('imu', imu_msg, queue_size=1000)

imu_message = imu_msg()
port = serial.Serial(serial_port, serial_baud, timeout=3)

port.write(b"$VNWRG,07,40*XX")

def euler_to_quaternion(a, b, c):

        z = math.radians(a)
        y = math.radians(b)
        x = math.radians(c)
        
        qx = np.sin(x/2) * np.cos(y/2) * np.cos(z/2) - np.cos(x/2) * np.sin(y/2) * np.sin(z/2)
        qy = np.cos(x/2) * np.sin(y/2) * np.cos(z/2) + np.sin(x/2) * np.cos(y/2) * np.sin(z/2)
        qz = np.cos(x/2) * np.cos(y/2) * np.sin(z/2) - np.sin(x/2) * np.sin(y/2) * np.cos(z/2)
        qw = np.cos(x/2) * np.cos(y/2) * np.cos(z/2) + np.sin(x/2) * np.sin(y/2) * np.sin(z/2)

        return [qw, qx, qy, qz]

	   
counter = 0 
while not rospy.is_shutdown():

    
    line = port.readline().decode("utf-8").strip()
    #To ensure that only VNYMR lines are printed

    if "$VNYMR" in line:
        
        data = line.split(",")

        yaw = float(data[1])  
        pitch = float(data[2])  
        roll = float(data[3]) 

        q = (euler_to_quaternion(yaw, pitch, roll))

        magx = float(data[4])
        magy = float(data[5])
        magz = float(data[6])

        accx = float(data[7])
        accy = float(data[8])
        accz = float(data[9])

        gyrox = float(data[10])
        gyroy = float(data[11])
        temp = data[12]
        gyroz_1 = temp.split("*")
        gyroz = float(gyroz_1[0])

        now = rospy.get_rostime()

        time_secs = now.secs
        time_nsecs = now.nsecs

        counter = counter + 1
        

        rawstring = ''.join(map(str,data))

        #IMU Message

        #Header
        imu_message.Header.seq = counter
        imu_message.Header.stamp.secs = time_secs 
        imu_message.Header.stamp.nsecs = time_nsecs
        imu_message.Header.frame_id = 'IMU1_Frame'


        #IMU
        imu_message.IMU.header.seq = counter
        imu_message.IMU.header.stamp.secs = time_secs 
        imu_message.IMU.header.stamp.nsecs = time_nsecs
        imu_message.IMU.header.frame_id = 'IMU1_Frame'
        
        imu_message.IMU.orientation.w = q[0]
        imu_message.IMU.orientation.x = q[1]
        imu_message.IMU.orientation.y = q[2]
        imu_message.IMU.orientation.z = q[3]

        imu_message.IMU.angular_velocity.x = gyrox
        imu_message.IMU.angular_velocity.y = gyroy
        imu_message.IMU.angular_velocity.z = gyroz

        imu_message.IMU.linear_acceleration.x = accx
        imu_message.IMU.linear_acceleration.y = accy 
        imu_message.IMU.linear_acceleration.z = accz

        #MagField
        imu_message.MagField.header.seq = counter
        imu_message.MagField.header.stamp.secs = time_secs 
        imu_message.MagField.header.stamp.nsecs = time_nsecs
        imu_message.MagField.header.frame_id = 'IMU1_Frame'

        imu_message.MagField.magnetic_field.x = magx
        imu_message.MagField.magnetic_field.y = magy
        imu_message.MagField.magnetic_field.z = magz 

        imu_message.raw_string = rawstring

        pub.publish(imu_message)

    line = None
