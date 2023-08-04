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
    # starttime = time.time()
    cnt = 1100
    time.sleep(2)
    while cam.isOpened():
        # endtime = time.time()
        if cnt>=1200:
            break
        ret,frame = cam.read()
        if ret:
            cv2.imwrite('/home/cquer/2023_qingzhou/src/2020.01.19NanoCSITest/red/red%4d.jpg'%(cnt),frame)
            print('red%4d.jpg'%(cnt))
            cnt += 1
        time.sleep(0.2)