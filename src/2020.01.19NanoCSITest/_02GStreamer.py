# -*- coding:utf-8 _*-
"""
@Author  : Yu Tao
@Time    : 2020/1/13 1:44 
"""
import cv2
# 3264 2464 21  CPU:27%, MEM:56.4
# 3264 1848 28  CPU:33%, MEM:55.4
# 1920 1080 30  CPU:30%, MEM:53.3
# 1280 720 60   CPU:45%, MEM:52.8
# 1280 720 120  CPU:75%, MEM:53.2

# 3264 2464

def gstreamer_pipeline(
		capture_width=1280,
		capture_height=960,
		display_width=1280,
		display_height=960,
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
#  gst-launch-1.0 nvarguscamerasrc ! 'video/x-raw(memory:NVMM),width=3820, height=2464, framerate=21/1, format=NV12' ! nvvidconv flip-method=0 ! 'video/x-raw,width=960, height=616' ! nvvidconv ! nvegltransform ! nveglglessink -e
