# -*- coding:utf-8 _*-
"""
@Author  : Yu Tao
@Time    : 2020/3/19 11:37 
"""
import numpy as np
np.set_printoptions(suppress=True, precision=4)
import cv2, time, socket, json
import multiprocessing as mp
from _02GStreamer import *

def getRoadLine(img):
	lower1=np.array([0,90,46])
	upper1=np.array([10,255,255])
	lower2=np.array([156,43,46])
	upper2=np.array([180,255,255])
	hsv_img=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
	mask1=cv2.inRange(hsv_img,lower1,upper1)
	mask2=cv2.inRange(hsv_img,lower2,upper2)

	masked_img1=cv2.bitwise_and(img,img,mask=mask1)
	masked_img2=cv2.bitwise_and(img,img,mask=mask2)
	masked_img=masked_img1+masked_img2

	gray_img=cv2.cvtColor(masked_img,cv2.COLOR_BGR2GRAY)
	ret,binary_img=cv2.threshold(gray_img,50,255,cv2.THRESH_BINARY)
	kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
	binary_img=cv2.morphologyEx(binary_img,cv2.MORPH_OPEN,kernel)
	binary_img=cv2.morphologyEx(binary_img,cv2.MORPH_CLOSE,kernel)

	return binary_img


def ImgRead(ImgQueue):
	# %% 从摄像头读取数据
	# cam = cv2.VideoCapture(0)
	cam = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
	if not cam.isOpened():
		print("Unable to open camera")
	else:
		print('Open camera success!')
		while True:
			ret, Img = cam.read()
			if not ret:
				break
			while not ImgQueue.empty():
				ImgQueue.get()
			ImgQueue.put(Img)
			cv2.imshow('ImgRead', Img)
			key = cv2.waitKey(5)
			if key == 27:
				break
		cam.release()

def vision():
	Frame = 0
	ImgQueue = mp.Queue()  # 先进先出队列，实现不同进程数据交互
	Mps = []
	Mps.append(mp.Process(target=ImgRead, args=(ImgQueue,)))
	[Mp.start() for Mp in Mps]
	# Mps[0].join()
	while ImgQueue.empty():
		pass
	while True:
		Key = input('Press s or S to save image:')
		if Key == 's' or Key == 'S':
			Img = ImgQueue.get()
			cv2.imwrite('%04d.jpg' % Frame, Img)
			print('Save image %04d.jpg success!' % Frame)
			Frame = Frame + 1
		elif Key == 'Q' or Key == 'q':
			break
	[Mp.terminate() for Mp in Mps]

if __name__ == '__main__':
	vision()
