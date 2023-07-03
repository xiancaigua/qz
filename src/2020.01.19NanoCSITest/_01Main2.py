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
			# cv2.imshow('ImgRead', Img)
			# key = cv2.waitKey(5)
			# if key == 27:
			# 	break
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
		Key = input('Press 2 or 1 to save image:')
		if Key == '1' or Key == '2':
			Img = ImgQueue.get()
			cv2.imwrite('/home/cquer/2023_qingzhou/src/2020.01.19NanoCSITest/biaoding/biaoding%4d.jpg' % Frame, Img)
			print('Save image %03d.jpg success!' % Frame)
			Frame = Frame + 1
		elif Key == '3':
			break
	[Mp.terminate() for Mp in Mps]

if __name__ == '__main__':
	vision()
