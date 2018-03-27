#!/usr/bin/python

import serial
import lcm
import time
import re

import sys
sys.path.append('/home/jeoker/Documents/EECE-5698/lab3/lcmtypes/')
from imu_struct import imu_msg

if __name__ == '__main__':
    ser = serial.Serial(port='/dev/ttyUSB1', baudrate=115200, timeout=1)
    lc = lcm.LCM()
    msg = imu_msg()
    t0 = time.time()
    counter = 0
    
    while True:
        line = ser.readline()
        splitline = line.split(',')
        tnow = time.time()
        if splitline[0] == '$VNYMR':
            counter += 1
            msg.yaw = float(splitline[1])
            msg.pitch = float(splitline[2])
            msg.roll = float(splitline[3])
            msg.magn_x = float(splitline[4])
            msg.magn_y = float(splitline[5])
            msg.magn_z = float(splitline[6])
            msg.accl_x = float(splitline[7])
            msg.accl_y = float(splitline[8])
            msg.accl_z = float(splitline[9])
            msg.gyro_x = float(splitline[10])#.replace("\x00",""))
            msg.gyro_y = float(splitline[11])
            msg.gyro_z = float(splitline[12][:-5])
            # print "x:%s y:%s z:%s" % (msg.accl_x, msg.accl_y, msg.accl_z)
            # time.sleep(1)
            # print "x:%s y:%s z:%s" % (msg.yaw, msg.pitch, msg.roll)
            
            lc.publish("imu_talker", msg.encode())
            
    ser.close()
