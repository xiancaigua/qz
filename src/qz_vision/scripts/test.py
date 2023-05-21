#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

import cv2


def gstreamer_pipeline(
		capture_width=3264,
		capture_height=2464,
		display_width=3264,
		display_height=2464,
		framerate=21,
		flip_method=0,
):
	return (
			"nvarguscamerasrc ! "
			"video/x-raw(memory:NVMM), "
			"width=(int)%d, height=(int)%d, "
			"format=(string)NV12, framerate=(fraction)%d/1 ! "
			"nvvidconv flip-method=%d ! "
			"video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
			"videoconvert ! "
			"video/x-raw, format=(string)BGR ! appsink"
			% (
				capture_width,
				capture_height,
				framerate,
				flip_method,
				display_width,
				display_height,
			)
	)


cam=cv2.VideoCapture(gstreamer_pipeline(flip_method=0),cv2.CAP_GSTREAMER) 
while cam.isOpened():
    ret,img=cam.read()
    if ret:
        img = cv2.resize(img, None, fx = 0.5, fy = 0.5, interpolation = cv2.INTER_CUBIC)
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        dict=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        parameters =  cv2.aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray_img, dict, parameters=parameters)
        print(ids)
        cv2.imshow('img',gray_img)
        key = cv2.waitKey(5)
        if key==27:
            break