#!/usr/bin/env python3
# coding=UTF-8
import rospy
from PIL import Image as PILImage
import io
import numpy as np
import cv2
from fly_pkg.srv import *
from fly_pkg.msg import t265_frame

if __name__ == '__main__':
    rospy.init_node('t265_vision_task', anonymous=True)
    min_disp = 0  # 最小视差值
    num_disp = 112 - min_disp
    max_disp = min_disp + num_disp
    while not rospy.is_shutdown():
        t265_image = rospy.wait_for_message("t265_vision_frame", t265_frame, timeout=None)
        # 将ROS消息转换为图像数据
        stream = io.BytesIO(t265_image.image.data)
        pil_image = PILImage.open(stream)
        image = np.array(pil_image)
        image = cv2.cvtColor(image,cv2.COLOR_GRAY2RGB)
        print((np.array(t265_image.distance_array.data).reshape((300,300))).shape)
        disparity = (np.array(t265_image.distance_array.data).reshape((300,300)) * num_disp /255)+min_disp
        nums = 0
        data = 0
        for i in range(140, 155):
            for j in range(145, 155):
                if disparity[i][j] > 0:
                    data += disparity[i][j]
                    nums += 1
        if nums == 0:
            data = -1
        else:
            data /= nums
            data = 462.6/data
        cv2.rectangle(image,(145,145),(145+10,145+10),(0,0,255),3)
        print(data)
        cv2.imshow('aa',image)
        cv2.waitKey(1)