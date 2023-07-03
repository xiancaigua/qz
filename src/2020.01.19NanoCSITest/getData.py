# -*- coding:utf-8 _*-
"""
@Author  : Zi Han
"""
import numpy as np
np.set_printoptions(suppress=True, precision=4)
import cv2, time, socket, json
import multiprocessing as mp
from _02GStreamer import *

if __name__ =="__main__":
    cam = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    photo_count=21
    while cam.isOpened():
        key = input('give me command')
        if key==1:
            for i in range(50):
                ret,frame = cam.read()
            if not ret:
                break
            cv2.imwrite('biaoding%4d.jpg'%photo_count,frame)
            print('biaoding_%4d.jpg is saved'%photo_count)
            photo_count+=1
        if key==2:
            break
        # time.sleep(5)    
        # while True:
            # time.sleep(5)
            # time.sleep(0.2)