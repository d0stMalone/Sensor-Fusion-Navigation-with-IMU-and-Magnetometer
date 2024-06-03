#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
import serial
import utm
from std_msgs.msg import Header
from gps_driver.msg import gps_msg
import sys
import argparse


if __name__ == '__main__':
    
    SENSOR_NAME = 'gps'
    rospy.init_node('gps_node')   
    
    port = rospy.get_param('driver1/port')
    serial_baud = rospy.get_param('~baudrate', 4800)
    sampling_rate = rospy.get_param('~sampling_rate',1.0)
    port = serial.Serial(port, serial_baud, timeout=3.)

    gps_pub = rospy.Publisher('gps', gps_msg, queue_size=10)
    gps_msg_fields = gps_msg()
    
    print("errir")
    try:
        
        
        while not rospy.is_shutdown():
            line = port.readline()
            print(line)
        
            if line:
                line_str = line.decode("utf-8").strip()
                
                if '$GPGGA' in line_str:
                    gps_data = line_str.split(",")
                    
                    try:                         
                        # Extract latitude, longitude, 
                        lat = float(gps_data[2])
                        lat_dd = int(lat / 100.0 )                 
                        lat_mm = float(lat - (lat_dd * 100))
                        new_lat = float(lat_dd + float(lat_mm/60))
                        new_lat *= 1 if gps_data[3] == 'N' else -1

                        lon = float(gps_data[4])
                        lon_dd = int(lon / 100.0)                  
                        lon_mm = float(lon - (lon_dd * 100))
                        new_lon = float(lon_dd + float(lon_mm/60))
                        new_lon *= 1 if gps_data[5] == 'E' else -1

                    except:
                        print("Trash")
                        continue
                                    
                    utm_latlon = utm.from_latlon(new_lat, new_lon)
                    
                    #Converting UTC to secs and nsecs
                    time = gps_data[1]
                    time_sec = (float(time[:2])*3600)+(float(time[2:4])*60)+float(time[4:6])
                    gps_msg_fields.header.stamp.secs = int(time_sec)
                    time_nsec = float(time[6:])*10e9
                    gps_msg_fields.header.stamp.nsecs = int(time_nsec)


                    # Create a ROS message 
                    gps_msg_fields.header.frame_id = 'GPS1_Frame'
                    gps_msg_fields.header.stamp.secs = int(time_sec)
                    gps_msg_fields.header.stamp.nsecs = int(time_nsec)

                    gps_msg_fields.Latitude = new_lat
                    gps_msg_fields.Longitude = new_lon
                    gps_msg_fields.Altitude = float(gps_data[9])
                    gps_msg_fields.UTM_northing = float(utm_latlon[1])
                    gps_msg_fields.UTM_easting = float(utm_latlon[0])
                    gps_msg_fields.Zone = int(utm_latlon[2])
                    gps_msg_fields.Letter = utm_latlon[3]
                    gps_msg_fields.HDOP = float(gps_data[8])
                    gps_msg_fields.UTC = gps_data[1]
                                                     
                    gps_pub.publish(gps_msg_fields)

    except rospy.ROSInterruptException:
            port.close()
            print('error')