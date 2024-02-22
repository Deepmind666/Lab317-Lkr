#!/usr/bin/python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.
# Python 2/3 compatibility
from __future__ import print_function

"""
This example shows how to use T265 intrinsics and extrinsics in OpenCV to
asynchronously compute depth maps from T265 fisheye images on the host.

T265 is not a depth camera and the quality of passive-only depth options will
always be limited compared to (e.g.) the D4XX series cameras. However, T265 does
have two global shutter cameras in a stereo configuration, and in this example
we show how to set up OpenCV to undistort the images and compute stereo depth
from them.

Getting started with python3, OpenCV and T265 on Ubuntu 16.04:

First, set up the virtual enviroment:

$ apt-get install python3-venv  # install python3 built in venv support
$ python3 -m venv py3librs      # create a virtual environment in pylibrs
$ source py3librs/bin/activate  # activate the venv, do this from every terminal
$ pip install opencv-python     # install opencv 4.1 in the venv
$ pip install pyrealsense2      # install librealsense python bindings

Then, for every new terminal:

$ source py3librs/bin/activate  # Activate the virtual environment
$ python3 t265_stereo.py        # Run the example
"""

# First import the library
import pyrealsense2 as rs

# Import OpenCV and numpy
import cv2
import numpy as np
from math import tan, pi

"""
In this section, we will set up the functions that will translate the camera
intrinsics and extrinsics from librealsense into parameters that can be used
with OpenCV.

The T265 uses very wide angle lenses, so the distortion is modeled using a four
parameter distortion model known as Kanalla-Brandt. OpenCV supports this
distortion model in their "fisheye" module, more details can be found here:

https://docs.opencv.org/3.4/db/d58/group__calib3d__fisheye.html
"""

"""
Returns R, T transform from src to dst
"""
def get_extrinsics(src, dst):
    extrinsics = src.get_extrinsics_to(dst)
    R = np.reshape(extrinsics.rotation, [3,3]).T
    T = np.array(extrinsics.translation)
    return (R, T)

"""
Returns a camera matrix K from librealsense intrinsics
"""
def camera_matrix(intrinsics):
    return np.array([[intrinsics.fx,             0, intrinsics.ppx],
                     [            0, intrinsics.fy, intrinsics.ppy],
                     [            0,             0,              1]])

"""
Returns the fisheye distortion from librealsense intrinsics
"""
def fisheye_distortion(intrinsics):
    return np.array(intrinsics.coeffs[:4])

# Set up a mutex to share data between threads
from threading import Lock
frame_mutex = Lock()
frame_data = {"left"  : None,
              "right" : None,
            "timestamp_ms":None
              }

class POSE_DATA:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.rotation_x = 0
        self.rotation_y = 0
        self.rotation_z = 0
        self.rotation_w = 0

# 创建对象并调用方法
pose = POSE_DATA()  #位置信息

"""
This callback is called on a separate thread, so we must use a mutex
to ensure that data is synchronized properly. We should also be
careful not to do much work on this thread to avoid data backing up in the
callback queue.
"""
def callback(frame):    #回调函数处理信息
    global frame_data,pose
    if frame.is_frameset():
        frameset = frame.as_frameset()
        f1 = frameset.get_fisheye_frame(1).as_video_frame()
        f2 = frameset.get_fisheye_frame(2).as_video_frame()

        left_data = np.asanyarray(f1.get_data())   #将鱼眼图片信息转为np数组类型
        right_data = np.asanyarray(f2.get_data())
        ts = frameset.get_timestamp()   #获得时间戳

        #设置互斥锁确保变量不会被其他地方使用   这里只用到图片和时间
        frame_mutex.acquire()
        frame_data["left"] = left_data
        frame_data["right"] = right_data
        frame_data["timestamp_ms"] = ts
        frame_mutex.release() #释放锁

    if frame.is_pose_frame():
        pose_frame = frame.as_pose_frame()
        pose_data = pose_frame.get_pose_data()
        position = pose_data.translation
        rotation = pose_data.rotation
        frame_mutex.acquire()
        pose.x = float(position.x)
        pose.y = float(position.y)
        pose.z = float(position.z)
        pose.rotation_x = float(rotation.x)
        pose.rotation_y = float(rotation.y)
        pose.rotation_z = float(rotation.z)
        pose.rotation_w = float(rotation.w)
        frame_mutex.release()  # 释放锁

if __name__ == "__main__":
    # Declare RealSense pipeline, encapsulating the actual device and sensors
    pipe = rs.pipeline()  #创建管道

    # Build config object and stream everything
    cfg = rs.config()     #使能

    cfg.enable_stream(rs.stream.fisheye,1,rs.format.y8)
    cfg.enable_stream(rs.stream.fisheye,2,rs.format.y8)
    cfg.enable_stream(rs.stream.pose,rs.format.six_dof)

    # Start streaming with our callback
    pipe.start(cfg, callback)   #开始传输

    try:
        # Set up an OpenCV window to visualize the results
        WINDOW_TITLE = 'Realsense'
        cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)  #创建一个可调整大小的窗口，可以手动调整窗口的大小


        window_size = 5  #窗口大小
        min_disp = 0     #最小视差值
        # must be divisible by 16
        num_disp = 112 - min_disp
        max_disp = min_disp + num_disp
        stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
                                       numDisparities = num_disp,
                                       blockSize = 16,
                                       P1 = 8*3*window_size**2,  #控制视差平滑度的参数
                                       P2 = 32*3*window_size**2,
                                       disp12MaxDiff = 1,
                                       uniquenessRatio = 10,
                                       speckleWindowSize = 100,
                                       speckleRange = 32)
        print(min_disp,num_disp,window_size)
        # Retreive the stream and intrinsic properties for both cameras
        profiles = pipe.get_active_profile()
        streams = {"left"  : profiles.get_stream(rs.stream.fisheye, 1).as_video_stream_profile(),
                   "right" : profiles.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()}
        #保存鱼眼图片的参数
        intrinsics = {"left"  : streams["left"].get_intrinsics(),
                      "right" : streams["right"].get_intrinsics()}


        # Print information about both cameras
        print("Left camera:",  intrinsics["left"])
        print("Right camera:", intrinsics["right"])

        # Translate the intrinsics from librealsense into OpenCV
        K_left  = camera_matrix(intrinsics["left"])
        D_left  = fisheye_distortion(intrinsics["left"])
        K_right = camera_matrix(intrinsics["right"])
        D_right = fisheye_distortion(intrinsics["right"])
        (width, height) = (intrinsics["left"].width, intrinsics["left"].height)

        # Get the relative extrinsics between the left and right camera
        (R, T) = get_extrinsics(streams["left"], streams["right"])

        # We need to determine what focal length our undistorted images should have
        # in order to set up the camera matrices for initUndistortRectifyMap.  We
        # could use stereoRectify, but here we show how to derive these projection
        # matrices from the calibration and a desired height and field of view

        # We calculate the undistorted focal length:
        #
        #         h
        # -----------------
        #  \      |      /
        #    \    | f  /
        #     \   |   /
        #      \ fov /
        #        \|/
        stereo_fov_rad = 120 * (pi/180)  # 90 degree desired fov  所需的视场角（field of view），以弧度表示。这里设置为90度。
        stereo_height_px = 300          # 300x300 pixel stereo output 期望的校正后图像高度，以像素为单位。这里设置为300像素。
        #计算校正后图像所需的焦距（focal length）
        stereo_focal_px = stereo_height_px/2 / tan(stereo_fov_rad/2)

        # We set the left rotation to identity and the right rotation
        # the rotation between the cameras
        R_left = np.eye(3)
        R_right = R

        # The stereo algorithm needs max_disp extra pixels in order to produce valid
        # disparity on the desired output region. This changes the width, but the
        # center of projection should be on the center of the cropped image
        #计算裁剪后得到的图片区域
        stereo_width_px = stereo_height_px + max_disp
        stereo_size = (stereo_width_px, stereo_height_px)
        stereo_cx = (stereo_height_px - 1)/2 + max_disp
        stereo_cy = (stereo_height_px - 1)/2

        # Construct the left and right projection matrices, the only difference is
        # that the right projection matrix should have a shift along the x axis of
        # baseline*focal_length
        P_left = np.array([[stereo_focal_px, 0, stereo_cx, 0],
                           [0, stereo_focal_px, stereo_cy, 0],
                           [0,               0,         1, 0]])
        P_right = P_left.copy()
        P_right[0][3] = T[0]*stereo_focal_px

        # Construct Q for use with cv2.reprojectImageTo3D. Subtract max_disp from x
        # since we will crop the disparity later
        Q = np.array([[1, 0,       0, -(stereo_cx - max_disp)],
                      [0, 1,       0, -stereo_cy],
                      [0, 0,       0, stereo_focal_px],
                      [0, 0, -1/T[0], 0]])

        # Create an undistortion map for the left and right camera which applies the
        # rectification and undoes the camera distortion. This only has to be done
        # once
        m1type = cv2.CV_32FC1
        (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(K_left, D_left, R_left, P_left, stereo_size, m1type)
        (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(K_right, D_right, R_right, P_right, stereo_size, m1type)
        undistort_rectify = {"left"  : (lm1, lm2),
                             "right" : (rm1, rm2)}

        mode = "stack"
        while True:
            # Check if the camera has acquired any frames
            frame_mutex.acquire()
            valid = frame_data["timestamp_ms"] is not None
            frame_mutex.release()

            # If frames are ready to process
            if valid:
                # Hold the mutex only long enough to copy the stereo frames
                frame_mutex.acquire()
                frame_copy = {"left"  : frame_data["left"].copy(),
                              "right" : frame_data["right"].copy()}
                frame_mutex.release()

                # Undistort and crop the center of the frames
                center_undistorted = {"left" : cv2.remap(src = frame_copy["left"],
                                              map1 = undistort_rectify["left"][0],
                                              map2 = undistort_rectify["left"][1],
                                              interpolation = cv2.INTER_LINEAR),
                                      "right" : cv2.remap(src = frame_copy["right"],
                                              map1 = undistort_rectify["right"][0],
                                              map2 = undistort_rectify["right"][1],
                                              interpolation = cv2.INTER_LINEAR)}

                # compute the disparity on the center of the frames and convert it to a pixel disparity (divide by DISP_SCALE=16)
                disparity = stereo.compute(center_undistorted["left"], center_undistorted["right"]).astype(np.float32) / 16.0

                # re-crop just the valid part of the disparity
                disparity = disparity[:,max_disp:]
                #print(disparity.shape)
                nums = 0
                data = 0
                for i in range(145,155):
                    for j in range(145,155):
                        if disparity[i][j]>0:
                            data+=disparity[i][j]
                            nums+=1
                if nums == 0:
                    data = -1
                else:
                    data/=nums
                #print("------------------------------------------------------------------")
                # if data!=-1:
                #     print("distance = ",100*9.525/data,"x = ",pose.x," y = ",pose.y," z = ",pose.z)
                #
                # else:
                #     print("distance = EEROR"," x = ",pose.x," y = ",pose.y," z = ",pose.z)
                # print(" o_x = ",pose.rotation_x," o_y = ",pose.rotation_y," o_z = ",pose.rotation_z," o_w = ",pose.rotation_w)
                print(pose.y)
                #print(pose_data['x'])
                #print(type(frame_data["timestamp_ms"] ))
                #print(data)
                # convert disparity to 0-255 and color it
                disp_vis = 255*(disparity - min_disp)/ num_disp #归一化

                cv2.imshow('1',disp_vis)
                disp_color = cv2.applyColorMap(cv2.convertScaleAbs(disp_vis,1), cv2.COLORMAP_JET)
                #print(disp_color.shape)
                color_image = cv2.cvtColor(center_undistorted["left"][:,max_disp:], cv2.COLOR_GRAY2RGB)
                cv2.rectangle(color_image, (145, 145), (155, 155), (0, 255, 0), 2)

                if mode == "stack":
                    cv2.imshow(WINDOW_TITLE, np.hstack((color_image, disp_color)))
                if mode == "overlay":
                    ind = disparity >= min_disp
                    color_image[ind, 0] = disp_color[ind, 0]
                    color_image[ind, 1] = disp_color[ind, 1]
                    color_image[ind, 2] = disp_color[ind, 2]
                    cv2.imshow(WINDOW_TITLE, color_image)

            key = cv2.waitKey(1)
            if key == ord('s'): mode = "stack"
            if key == ord('o'): mode = "overlay"
            if key == ord('q') or cv2.getWindowProperty(WINDOW_TITLE, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        pipe.stop()