#!/usr/bin/env python
import rospy
import serial
import utm
import numpy as np
from imu_driver.msg import imu_msg
from imu_driver.srv import convert_to_quaternion
import datetime


def convert_to_quaternion_client(roll, pitch, yaw):
    try:
        # Wait for the "convert_to_quaternion" service to become available
        rospy.wait_for_service("convert_to_quaternion")

        # Create a service proxy to call the service
        conversion = rospy.ServiceProxy('convert_to_quaternion', convert_to_quaternion)
        
        # Call the service with the provided Euler angles
        response = conversion(roll, pitch, yaw)
        return (response.qx, response.qy, response.qz, response.qw)
    
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return None

def talker():
    pub = rospy.Publisher('imu',imu_msg, queue_size=10)
    rospy.init_node('imu_driver')
    rate = rospy.Rate(1)
    port = rospy.get_param('driver/port')
    serial_baud = rospy.get_param('~baudrate',115200)
    port = serial.Serial(port, serial_baud, timeout=3)
    imu_msg1 = imu_msg()
    
    while not rospy.is_shutdown():
        line = port.readline()
        print(line)
        if "$VNYMR" in str(line):
            new_data = str(line).split(",")
            print(new_data)


            yaw = float(new_data[1])
            pitch = float(new_data[2])
            roll = float(new_data[3])

            print("Roll,Pitch,Yaw:",roll,pitch,yaw)
        
            (qx, qy, qz, qw) = quaternion = convert_to_quaternion_client(roll, pitch, yaw)

            

            magx = float(new_data[4]) #in Gauss
            magy = float(new_data[5]) #in Gauss
            magz = float(new_data[6]) #in Gauss

            #convert to tesla 
            magx_t = magx*0.0001
            magy_t = magy*0.0001
            magz_t = magz*0.0001

            accx = float(new_data[7])
            accy = float(new_data[8])
            accz = float(new_data[9])

            gyx = float(new_data[10])
            gyy = float(new_data[11])
            gyz = float(new_data[12][0:9])


            #If the coordinates arent received, stop the code
            if new_data[2]=='':
                print("Data not being received")
                break
            
            #Publish to msg
            imu_msg1.header.frame_id = 'IMU1_Frame'
            imu_msg1.header.stamp = rospy.Time.now()

            imu_msg1.imu.orientation.x = qx
            imu_msg1.imu.orientation.y = qy
            imu_msg1.imu.orientation.z = qz
            imu_msg1.imu.orientation.w = qw
            imu_msg1.imu.angular_velocity.x = gyx
            imu_msg1.imu.angular_velocity.y = gyy   
            imu_msg1.imu.angular_velocity.z = gyz
            imu_msg1.imu.linear_acceleration.x = accx
            imu_msg1.imu.linear_acceleration.y = accy   
            imu_msg1.imu.linear_acceleration.z = accz
            imu_msg1.mag_field.magnetic_field.x = magx_t
            imu_msg1.mag_field.magnetic_field.y = magy_t
            imu_msg1.mag_field.magnetic_field.z = magz_t
            #imu_msg.raw_values = str(new_data)
            pub.publish(imu_msg1)   
            #print(rospy.Time.now())

if __name__ == '__main__':
    try:
        talker()

    except rospy.ROSInterruptException:
        pass
