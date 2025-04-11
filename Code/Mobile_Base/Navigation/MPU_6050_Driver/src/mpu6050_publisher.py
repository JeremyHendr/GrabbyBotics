#!/usr/bin/env python2

# -------------------------------------- #
# Project: Grabby the Warehouse robot
# Date: 14.01.2025
# -------------------------------------- #

import rospy
import smbus
import numpy as np
from time import sleep  

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_about_axis

class mpu6050:
    # Global Variables
    GRAVITIY_MS2 = 9.80665
    address = None
    bus = smbus.SMBus(1)

    # Scale modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0
 
    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18
 
    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    # MPU-6050 Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C
 
    SELF_TEST_X = 0x0D
    SELF_TEST_Y = 0x0E
    SELF_TEST_Z = 0x0F
    SELF_TEST_A = 0x10
 
    ACCEL_XOUT0 = 0x3B
    ACCEL_XOUT1 = 0x3C
    ACCEL_YOUT0 = 0x3D
    ACCEL_YOUT1 = 0x3E
    ACCEL_ZOUT0 = 0x3F
    ACCEL_ZOUT1 = 0x40
 
    TEMP_OUT0 = 0x41
    TEMP_OUT1 = 0x42
 
    GYRO_XOUT0 = 0x43
    GYRO_XOUT1 = 0x44
    GYRO_YOUT0 = 0x45
    GYRO_YOUT1 = 0x46
    GYRO_ZOUT0 = 0x47
    GYRO_ZOUT1 = 0x48
 
    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B

    def __init__(self, address):
        self.address = address

        # Start MPU-6050
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

    # I2C communication
    def read_i2c_word(self, register):
        # Read data from register
        high_bit = self.bus.read_byte_data(self.address, register)
        low_bit = self.bus.read_byte_data(self.address, register + 1)

        value = (high_bit << 8) + low_bit

        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value
        
    # Set accelerometer range
    def set_accel_range(self, accel_range):
        # Set register to 0x00
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)
        # Write new range value to ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)
    
    # Set gyrometer range
    def set_gyro_range(self, gyro_range):
        # Set register to 0x00
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)
        # Write new range value to GYRO_CONFIG register
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)

    # Read accelerometer range
    def read_accel_range(self, raw = False):
        # Get the raw value
        raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)
 
        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.ACCEL_RANGE_2G:
                return 2
            elif raw_data == self.ACCEL_RANGE_4G:
                return 4
            elif raw_data == self.ACCEL_RANGE_8G:
                return 8
            elif raw_data == self.ACCEL_RANGE_16G:
                return 16
            else:
                return -1
            
    # Read gyrometer range
    def read_gyro_range(self, raw = False):
        # Get the raw value
        raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)
 
        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.GYRO_RANGE_250DEG:
                return 250
            elif raw_data == self.GYRO_RANGE_500DEG:
                return 500
            elif raw_data == self.GYRO_RANGE_1000DEG:
                return 1000
            elif raw_data == self.GYRO_RANGE_2000DEG:
                return 2000
            else:
                return -1


    # Get accelerometer data
    def get_accel_data(self):
        # Read the data from the MPU-6050
        x = self.read_i2c_word(self.ACCEL_XOUT0)
        y = self.read_i2c_word(self.ACCEL_YOUT0)
        z = self.read_i2c_word(self.ACCEL_ZOUT0)
 
        accel_scale_modifier = None
        accel_range = self.read_accel_range(True)
 
        if accel_range == self.ACCEL_RANGE_2G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
        elif accel_range == self.ACCEL_RANGE_4G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
        elif accel_range == self.ACCEL_RANGE_8G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
        elif accel_range == self.ACCEL_RANGE_16G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
        else:
            print("Unkown range - accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G")
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
 
        x = x / accel_scale_modifier
        y = y / accel_scale_modifier
        z = z / accel_scale_modifier
 
        return {'x': x, 'y': y, 'z': z}
       
    # Get gyrometer data
    def get_gyro_data(self):
        # Read the raw data from the MPU-6050
        x = self.read_i2c_word(self.GYRO_XOUT0)
        y = self.read_i2c_word(self.GYRO_YOUT0)
        z = self.read_i2c_word(self.GYRO_ZOUT0)
 
        gyro_scale_modifier = None
        gyro_range = self.read_gyro_range(True)
 
        if gyro_range == self.GYRO_RANGE_250DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
        elif gyro_range == self.GYRO_RANGE_500DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
        elif gyro_range == self.GYRO_RANGE_1000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
        elif gyro_range == self.GYRO_RANGE_2000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
        else:
            print("Unkown range - gyro_scale_modifier set to self.GYRO_SCALE_MODIFIER_250DEG")
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
 
        x = x / gyro_scale_modifier
        y = y / gyro_scale_modifier
        z = z / gyro_scale_modifier
 
        return {'x': x, 'y': y, 'z': z}
 
    # Get accelerometer and gyrometer data
    def get_all_data(self):
        accel = get_accel_data()
        gyro = get_gyro_data()
 
        return [accel, gyro]


# ROS integration
def mpu6050_publisher(timer_event):    
    # Define message
    imu_msg = Imu()
    imu_msg.header.frame_id = "imu"

    # Get accelerometer and gyrometer data
    accel_data = mpu.get_accel_data()
    gyro_data = mpu.get_gyro_data()

    # Get acceleration data
    accel_x = accel_data['x']
    accel_y = accel_data['y']
    accel_z = accel_data['z']

    # Calculate quaternion representing the orientation
    accel = accel_x, accel_y, accel_z
    ref = np.array([0, 0, 1])
    acceln = accel / np.linalg.norm(accel)
    axis = np.cross(acceln, ref)
    angle = np.arccos(np.dot(acceln, ref))
    orientation = quaternion_about_axis(angle, axis)

    # Get gyrometer data
    gyro_x = gyro_data['x']
    gyro_y = gyro_data['y']
    gyro_z = gyro_data['z']

    # Populate IMU message
    o = imu_msg.orientation
    o.x, o.y, o.z, o.w = orientation

    # Set linear acceleration
    imu_msg.linear_acceleration.x = accel_x
    imu_msg.linear_acceleration.y = accel_y
    imu_msg.linear_acceleration.z = accel_z

    # Set angular velocity
    imu_msg.angular_velocity.x = gyro_x
    imu_msg.angular_velocity.y = gyro_y
    imu_msg.angular_velocity.z = gyro_z

    imu_msg.header.stamp = rospy.Time.now()

    # Optionally, set covariance matrices
    imu_msg.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    imu_msg.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    imu_msg.linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

    imu_pub.publish(imu_msg)


if __name__ == "__main__":
    rospy.init_node('mpu6050_publisher', anonymous=True)

    mpu = mpu6050(0x68)
    
    imu_pub = rospy.Publisher('/imu_data', Imu, queue_size=10)
    imu_timer = rospy.Timer(rospy.Duration(0.02), mpu6050_publisher)
    rospy.spin()