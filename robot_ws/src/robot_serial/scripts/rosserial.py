#!/usr/bin/env python3
import serial

import rospy
from geometry_msgs.msg import Twist
from robot_msgs.msg import lowlevel

lowlevel_ = lowlevel()

SerialPort = serial.Serial("/dev/ttyACM0", baudrate=115200, timeout=1)
SerialPort.close()
SerialPort.open()

def commandVelocityCallback(cmd_vel_msg):
    linear_x_ = cmd_vel_msg.linear.x
    linear_y_ = cmd_vel_msg.linear.y
    angular_z_ = cmd_vel_msg.angular.z

    if linear_x_ > 0.5: 
        linear_x_ = 0.5
    elif linear_x_ < -0.5:
        linear_x_ = -0.5
    
    if linear_y_ > 0.5: 
        linear_y_ = 0.5
    elif linear_y_ < -0.5:
        linear_y_ = -0.5

    string_ROS_STM32 = "CD" + "," + str(linear_x_) + "," + str(linear_y_) + "," + str(angular_z_) + ",\r\n"
    SerialPort.write(string_ROS_STM32.encode())

def initData():
    lowlevel_.header.frame_id = "base_link"

    lowlevel_.accelerometer.x = 0.0
    lowlevel_.accelerometer.y = 0.0
    lowlevel_.accelerometer.z = 0.0
    lowlevel_.gyroscope.x = 0.0
    lowlevel_.gyroscope.y = 0.0
    lowlevel_.gyroscope.z = 0.0
    lowlevel_.velocity.x = 0.0
    lowlevel_.velocity.y = 0.0
    lowlevel_.velocity.theta = 0.0
    lowlevel_.voltage.data = 0.0

    rospy.loginfo("Initial message data.")

def initSerial():
    rospy.loginfo("Initial rosserial /dev/ttyACM0")

def main():
    rospy.init_node('robot_serial_node', anonymous=True)
    
    initData()
    initSerial()

    rate = rospy.Rate(30)

    cmd_vel_sub_ = rospy.Subscriber('/cmd_vel', Twist, commandVelocityCallback)
    rospy.loginfo("ROS topic subscriber /cmd_vel")

    lowlevel_data_pub_ = rospy.Publisher('/robot/lowlevel/data', lowlevel, queue_size=10)
    rospy.loginfo("ROS topic publisher /robot/lowlevel/data")

    while not rospy.is_shutdown():
        STM32_string_data = 0
        string_rev = ""
        while len(string_rev) < 13:
            string_rev = SerialPort.read_until('\n'.encode()).decode('utf-8')

        STM32_string_data = string_rev.split(',')

        if STM32_string_data[0] == "RD" and "RD" not in STM32_string_data[1] and "RD" not in STM32_string_data[2] and "RD" not in STM32_string_data[3] and "RD" not in STM32_string_data[4] and "RD" not in STM32_string_data[5] and "RD" not in STM32_string_data[6] and "RD" not in STM32_string_data[7] and "RD" not in STM32_string_data[8] and "RD" not in STM32_string_data[9] and "RD" not in STM32_string_data[10]:
            lowlevel_.accelerometer.x = float(STM32_string_data[1])
            lowlevel_.accelerometer.y = float(STM32_string_data[2])
            lowlevel_.accelerometer.z = float(STM32_string_data[3])
            lowlevel_.gyroscope.x = float(STM32_string_data[4])
            lowlevel_.gyroscope.y = float(STM32_string_data[5])
            lowlevel_.gyroscope.z = float(STM32_string_data[6])
            lowlevel_.velocity.x = float(STM32_string_data[7])
            lowlevel_.velocity.y = float(STM32_string_data[8])
            lowlevel_.velocity.theta = float(STM32_string_data[9])
            lowlevel_.voltage.data = float(STM32_string_data[10])
        lowlevel_.header.stamp = rospy.Time.now()
        lowlevel_data_pub_.publish(lowlevel_)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass