#!/usr/bin/python

import serial
import lcm
import time
import utm

import sys
sys.path.append('/home/jeoker/Documents/EECE-5698/lab3/lcmtypes/')
from gps_struct import gpgga_msg

def raw2degree(raw_lat, raw_lon):
    lat_degree = raw_lat / 100.0
    lon_degree = raw_lon / 100.0
    lat_minute = float(raw_lat) % 100.0 / 60.0
    lon_minute = float(raw_lon) % 100.0 / 60.0
    
    new_lat = lat_minute + float(lat_degree)
    new_lon = -(lon_minute + float(lon_degree))
    return (new_lat, new_lon) #return the negative value for west longitude

lc = lcm.LCM()
msg = gpgga_msg()

ser = serial.Serial(port='/dev/ttyUSB0', baudrate=4800, timeout=1)

print "Establishing Connection.."
try:
    while True:
        line = ser.readline()
        splitline = line.split(',')
        if splitline[0] == '$GPGGA':
            if splitline[2] == '':
                print '[Wiating for signals...] Please go to open area '
            else:
                msg.lat, msg.lon = raw2degree(float(splitline[2]), float(splitline[4]))
                utm_pack = utm.from_latlon(msg.lat, msg.lon)
                msg.utc_time = float(splitline[1])
                msg.alt      = float(splitline[9])
                msg.utm_x = utm_pack[0]
                msg.utm_y = utm_pack[1]
                # print msg.lat, msg.lon, msg.alt
                # print msg.utm_x, msg.utm_y
                # print 
                lc.publish("gps_talker", msg.encode())

except KeyboardInterrupt:
    print "\nConnection Closed."
    pass
    ser.close()
    
ser.close()
    

