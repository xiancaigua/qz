#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-
import cv2
import numpy as np

thread=1000#3000
thread_notGreen = 9000
# thread_notGreen = 3500
color_dict = {
    'Green':[[60, 43, 150], [80, 255, 255]],
    'RandY':[[0, 50,46], [10, 255, 255]]
}


def cb(config,level):
    global color_dict,thread
    color_dict["RandY"][0][0]=config.H1
    color_dict["RandY"][0][1]=config.S1
    color_dict["RandY"][0][2]=config.V1
    color_dict["RandY"][1][0]=config.H2
    color_dict["RandY"][1][1]=config.S2
    color_dict["RandY"][1][2]=config.V2
    thread = config.thread
    print("[VISION]-----RandY: h1:%d,s1:%d,v1:%d h2:%d,s2:%d,v2:%d"%(
        color_dict["RandY"][0][0],
        color_dict["RandY"][0][1],
        color_dict["RandY"][0][2],
        color_dict["RandY"][1][0],
        color_dict["RandY"][1][1],
        color_dict["RandY"][1][2]
    ))
    return config

def shiftArea(img):
    row, column, _ = img.shape
    temp_img = img[int(row*0.2):int(row), int(column*0.2):int(column*0.8)]
    return temp_img


# 根据传入的color，从color_dict中获取对应上下限提取对应的颜色
def getColorArea(img, color):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_img, lowerb=np.array(color_dict[color][0]), upperb=np.array(color_dict[color][1])) 
    color_img = cv2.bitwise_and(img,img, mask = mask)
    return color_img
