#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

import cv2
from cv_bridge import CvBridge,CvBridgeError
import rospy
import sys
import queue
import numpy as np
import signal
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

"""
This is programmed by xcg
as for why all the explannation is in English?
it is because written on the car, and the computer do not have CHINSES
sadly
"""

"""
Globals
"""
color_dict = {
	'Blue':[[90, 53, 60], [140, 157, 145]],
	'Green':[[46, 50, 160], [92, 255, 255]],
	'RandY':[[0, 30, 50], [35, 255, 255]]
}

"""
The class of ROS 
In this class  we will ros_init
we will create ros publisher and subscriber

"""
# class ROS_INIT_CLASS():
# 	def __init__(self):
# 		self.move_pub=rospy.Publisher("/qz_cmd_vel",Twist,queue_size=5)
# 	def __call__(self):
# 		move_msg=Twist()
# 		rospy.timer.sleep(rospy.Duration(2))
# 		rospy.loginfo("Robot will move backward for 3s.")
# 		move_msg.linear.x = -0.3
# 		move_msg.angular.z = 0
# 		self.move_pub.publish(move_msg)
# 		rospy.timer.sleep(rospy.Duration(3))

# 		rospy.loginfo("Robot will move forward for 3s.")
# 		move_msg.linear.x = 0.3
# 		move_msg.angular.z = 0
# 		self.move_pub.publish(move_msg)
# 		rospy.timer.sleep(rospy.Duration(3))

# 		rospy.loginfo("Robot stop.")
# 		move_msg.linear.x = 0
# 		move_msg.angular.z = 0
# 		self.move_pub.publish(move_msg)
# 		rospy.timer.sleep(rospy.Duration(2))



"""
Use Gstreamer to open the CSI camera
"""
def gstreamer_pipeline(
		capture_width=3264,
		capture_height=2464,
		display_width=640,
		display_height=480,
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

#  To make what camera get visualize, but you can only use this when the car is connected to screen
def show(cam):
	if not cam.isOpened():
		print("A oh.......")
	else:
		print("Camera is OPENED")
		while True:
			ret,frame=cam.read()
			if not ret:
				break
			# print(frame.shape)
			frame=getColorArea(frame,'Blue')
			frame=handleImg(frame,4,10)
			cv2.imshow("ImgShow",frame)
			key=cv2.waitKey(1)
			if key==27:
				break
		cam.release()

"""
utils
"""



#  extract specific color from the picture
def getColorArea(img,color):
	hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	mask=cv2.inRange(hsv_img,lowerb=np.array(color_dict[color][0]),upperb=np.array(color_dict[color][1]))
	color_img=cv2.bitwise_and(img,img,mask=mask)
	return color_img

#  binary the picture
#  wipe the black dots and white dots
def handleImg(img,kernel_size,low):
	gray_Img=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	gray_Img=gray_Img[280:,:]
	_,img_thresh = cv2.threshold(gray_Img,low,255,cv2.THRESH_BINARY)
	kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(kernel_size,kernel_size))
	img_thresh=cv2.morphologyEx(img_thresh,cv2.MORPH_OPEN,kernel)
	img_thresh=cv2.morphologyEx(img_thresh,cv2.MORPH_CLOSE,kernel)
	return img_thresh

"""
1. Pixel value erzhihua
2.Calculate the weight of  Picture
3.Calculate related move message
"""
def HandleRoadLine(img):
	#  step 1
	binary_img=handleImg(img,10,100)

	#  step 2 3
	angle=getLineCenterandAngle(binary_img)

	return angle

def getROI(img,x,y):
	h, w = img.shape[:2]

	poly = np.array([
		[(0, h - y), (w - x, h - y), (w - x, h), (0, h)]
	])

	if len(img.shape) > 2:
		gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		mask = np.zeros_like(gray_img)
	else:
		mask = np.zeros_like(img)
	cv2.fillPoly(mask, poly, 255)
	masked_img = cv2.bitwise_and(img, img, mask = mask)
	return masked_img

def getLineCenterandAngle(img):
	histogram=np.sum(img[:,:],axis=0)
	midpoint=int(histogram.shape[0]/2)
	leftx_base=np.argmax(histogram[:midpoint])
	rightx_base=np.argmax(histogram[midpoint+30:])
	if leftx_base!=0 and rightx_base!=0:
		base=leftx_base+30
	else:
		base=rightx_base+midpoint+30

	RoadLineROI=getROI(img,img.shape[0]-base,100)

	# cv2.imwrite("/home/cquer/2023_qingzhou/src/qz_vision/ROI.jpg",RoadLineROI)
	
	contours,hierarchy=cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	if len(contours)>0:
		M=cv2.moments(contours[0])
		cx=int(M['m10']/(M['m00']+float("1e-5")))
		cy=int(M['m01']/(M['m00']+float("1e-5")))
		print(cx,cy)

		if cx==0 and cy==0:
			angle=0
		else:
			angle=np.degrees(np.arctan(float(160-cx)/float(cy-26)))
		return angle

"""
1.get the aruco code
2.get the ROI
3.get color
"""

def HandleTrafficLight(img):
	green_count=0
	randy_count=0
	#  step 1
	aruco_id,corners,gray_img=detectAruco(img)
	#  step 2
	if aruco_id==1:
		if len(corners)>0:
			x=corners[0][0][:,0]
			x=corners[0][0][:,1]

			max_x, min_x, min_y = int(max(x)), int(min(x)), int(min(y))   # 提取aruco码上面的部分，即红绿灯部分
			poly = np.array([
				[(min_x, 0), (max_x, 0), (max_x, min_y), (min_x, min_y)]
			])
			mask=np.zeros_like(gray_img)
			cv2.fillPoly(mask,poly,255)
			light_img=cv2.bitwise_and(img,img,mask=mask)
			#  get color
			img_Lthresh=handleImg(light_img,2,180)
			cen_img=cv2.bitwise_and(light_img,light_img,mask=img_Lthresh)
			green_Img=getColorArea(cen_img,'Green')
			Rand_Img=getColorArea(cen_img,'RandY')

			if green_Img.any():
				green_count+=1
			if Rand_Img.any():
				randy_count+=1

			if green_count==8:
				color_flag=1
			elif randy_count==8:
				color_flag=2
	return color_flag

def detectAruco(img):
	img=cv2.resize(img,None,fx=0.5,fy=0.5,interpolation=cv2.INTER_CUBIC)
	gray_img=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	corners,ids,rejectedImgPoints=cv2.aruco.detectMarkers(gray_img,
			cv2.aruco.getPredefinedDictionary(aruco.DICT_4X4_1000),
			parameters=cv2.aruco.DetectorParameters_create()
			)
	if ids is not None:
		return ids[0][0],corners,gray_img

"""
@input img: cv2.Image 
	-directly captured by camera
@output point: cv2.Position 
	-the position of the center line
1. erzhihua
2. cut off both sides
2. extract black block
"""
def blackBlock(img):
	# cut off img
	# img_cut = getROI(img)
	img_cut=img
	# erzhihua here
	colorFlip = cv2.cvtColor(img_cut, cv2.COLOR_BGR2HSV)
	lower_bound = (0, 0, 0)
	upper_bound = (10, 50, 255)
	mask = cv2.inRange(colorFlip, lower_bound, upper_bound)
	# Dilation & Opening
	kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
	mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
	mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
	# find positon
	positions = cv2.findNonZero(mask) # containing all black blocks
	# find the largest one

	return positions


if __name__=="__main__":

	rospy.init_node("qz_vision")
	move_pub=rospy.Publisher("/qz_cmd_vel",Twist,queue_size=5)
	cam=cv2.VideoCapture(gstreamer_pipeline(flip_method=0),cv2.CAP_GSTREAMER) 
	fourcc = cv2.VideoWriter_fourcc(*'XVID')
	out = cv2.VideoWriter('/home/cquer/2023_qingzhou/src/qz_vision/output.avi',fourcc, 30.0, (640,480))

	cou1=0
	outbreak=0
	while not rospy.is_shutdown():
		while cam.isOpened():	
			ret,img=cam.read()
			if img is not None:
					
					# show_img=handleImg(img,10,100)
				frame = cv2.flip(img,1)
				out.write(frame)

				angle=HandleRoadLine(img)
					# cv2.imwrite("/home/cquer/2023_qingzhou/src/qz_vision/Picture4.jpg",show_img)
					# print("Picture Saved Successfully! Picture4")
				if angle:
					pass
				else:
					outbreak+=1
				
				msg=Twist()
				msg.linear.x=0.2
				msg.angular.z=angle
				
				if outbreak==5:
					msg.linear.x=0
					msg.angular.z=0
				move_pub.publish(msg)




