#!/usr/bin/env python
# coding=UTF-8
import rospy
from fly_pkg.srv import *

if __name__ == "__main__":
    rospy.init_node("vision_task")
    client = rospy.ServiceProxy("vision_data", vision_srv)
    client.wait_for_service()
    vision_request = vision_srvRequest()
    vision_request.x[0] = 1
    vision_request.x[1] = 2
    vision_request.x[2] = 3
    vision_request.number = 3
    vision_open = 0
    waiting_rate = rospy.Rate(10)  # 10hz
    transform_rate = rospy.Rate(50)  # 50hz
    while not rospy.is_shutdown():
        if  not vision_open:
            vision_respond = client.call(vision_request)
            vision_open = vision_respond.need_vision
            rospy.loginfo("等待vision_open中")
            waiting_rate.sleep()  # 等待
        else:
            vision_request.x.append(1)
            vision_respond = client.call(vision_request)
            vision_open = vision_respond.need_vision
            rospy.loginfo("发送数据中")
            transform_rate.sleep()
