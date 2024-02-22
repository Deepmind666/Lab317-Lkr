#!/usr/bin/env python
# coding=UTF-8
import rospy
import serial
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
import binascii
import math
offset_y=0.115
pos_data =  Imu()
def get_true_pos_z(pitch,roll,distance):
    return math.sqrt((distance**2)/(1+(math.tan(pitch)**2)+(math.tan(roll)**2)))-offset_y*math.sin(pitch)

def main():
    # 设置串口参数
    global pos_data
    rospy.init_node("get_light")
    pub = rospy.Publisher("get_light", Float64 , queue_size=100) #位置发送
    light_pos_z = Float64()
    ser = serial.Serial('/dev/ttyACM0', 921600, timeout=None)
    light_data = []
    rate = rospy.Rate(80)  # 80hz
    last_time = rospy.Time.now()
    while not rospy.is_shutdown():
        temp = ser.read(1)
        if len(temp)<1:
            continue
        light_data.append(int(str(binascii.b2a_hex(temp)),16))  #串口转16进制
        if(light_data[0]!=0x57):
            light_data = []
        if(len(light_data) == 4):
            if(light_data[1] != 0x00 or light_data[2] != 0xff or light_data[3] != 0x00):
                light_data = []
        if(len(light_data)==16): #数据接收完毕
            sum = 0
            for i in light_data[:len(light_data)-1]:
                sum+=i
            sum = sum & 0xff
            if light_data[15] == sum:
                distance = float((light_data[8]| (light_data[9]<<8)|(light_data[10]<<16)))/1000
                if(rospy.Time.now() - last_time > rospy.Duration(0.025)):
                    ser.flushInput()
                    last_time = rospy.Time.now()
                    pos_data = rospy.wait_for_message("mavros/imu/data", Imu, timeout=None)
                IMU_q = []
                IMU_q.append(pos_data.orientation.x)
                IMU_q.append(pos_data.orientation.y)
                IMU_q.append(pos_data.orientation.z)
                IMU_q.append(pos_data.orientation.w)
                mavros_ENU_pitch = math.asin(2 * (IMU_q[0] * IMU_q[2] - IMU_q[3] * IMU_q[1]))
                mavros_ENU_roll  = math.atan2(2 * (IMU_q[0] * IMU_q[3] + IMU_q[1] * IMU_q[2]),1 - 2 * (IMU_q[2] * IMU_q[2] + IMU_q[3] * IMU_q[3]))
                if (mavros_ENU_roll > 0):
                    mavros_ENU_roll = math.pi - mavros_ENU_roll
                else:
                    mavros_ENU_roll = -math.pi - mavros_ENU_roll
                light_pos_z.data = get_true_pos_z(mavros_ENU_pitch,mavros_ENU_roll,distance)
                pub.publish(light_pos_z)
                print(light_pos_z.data,mavros_ENU_pitch,distance)
                rate.sleep()  # 等待
            light_data = []




if __name__ == "__main__":
    main()
