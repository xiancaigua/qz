#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

import cv2
import numpy as np

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

# S弯绿色块   # 81  46  60
Slowergreen = np.array([50,40,45])
Shighgreen = np.array([90,255,255])
def color_seperate(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   #对目标图像进行色彩空间转换
    lower_hsv = Slowergreen          #设定green下限
    upper_hsv = Shighgreen        #设定green上限
    mask = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)  #依据设定的上下限对目标图像进行二值化转换，低于lower,高于upper都变成0，在中间为255
    dst = cv2.bitwise_and(image, image, mask=mask)    #将image的mask区域提取出来给dst,即找到蓝色区域并赋值给dst
    return dst

def region_of_interest(r_image):
    h = r_image.shape[0] #图片的高
    w = r_image.shape[1] #图片的宽
    ImgH_=180  #90
    gray = cv2.cvtColor(r_image, cv2.COLOR_BGR2GRAY) #转化为灰度图

    poly = np.array([
        [(0, h-ImgH_), (w, h-ImgH_), (w, h-20), (0, h-20)]  #将数组转化为矩阵，四个为长方形的四个顶点，从左上角开始顺时针,最下面竖着100个像素的图 #原来是90
    ])
    mask = np.zeros_like(gray) #输入为矩阵gray，输出为形状和gray一致的矩阵，其元素全部为黑色0
    cv2.fillPoly(mask, poly, 255)   #将mask的poly部分填充为白色255
    masked_image = cv2.bitwise_and(r_image,r_image, mask=mask)  #将r_image的mask区域提取出来给masked_image
    return masked_image
def color_seperate_1(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   #对目标图像进行色彩空间转换
    lower_hsv = np.array([100, 43, 46])          #设定红色下限
    upper_hsv = np.array([160, 255, 255])        #设定红色上限
    mask = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)  #依据设定的上下限对目标图像进行二值化转换，低于lower,高于upper都变成0，在中间为255
    dst = cv2.bitwise_and(image, image, mask=mask)    #将image的mask区域提取出来给dst,即找到红色区域并赋值给dst
    # cv2.imshow('red', dst)  #查看红色区域的图像
    # # cv2.waitKey(3)
    return dst
FOV_w=105
flag = 0
pianyi_before = 0
cnt=0
cam=cv2.VideoCapture(gstreamer_pipeline(flip_method=0),cv2.CAP_GSTREAMER) 
while cam.isOpened():
    ret,Img = cam.read()
    if ret:
        Img_2  =  cv2.resize(Img,(640,480))
        Img_2= Img_2[3: 475,3:635]
        Img_2 = cv2.resize(Img_2,(640,480))
        (img_w, img_h) = Img_2.shape[:2] #获取传入图片的长与宽   w是高 h是宽


        lane_img=Img_2.copy()
        cropped_img=region_of_interest(lane_img)
        cropped_img_1=cropped_img.copy()
        cropped_img = color_seperate(cropped_img)  
        gray_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
        ret, img_thresh = cv2.threshold(gray_img,10, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4, 4)) 
        img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_CLOSE, kernel)
        img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
        contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if (len(contours)>0):
            con_num = len(contours) 
            contour1 = []
            for c1 in range(len(contours)): #遍历每一个轮廓
                for c2 in range(len(contours[c1])): #遍历每一个轮廓的轮廓上的点
                    contour1.append(contours[c1][c2]) #将每一个轮廓的每一个点都排列起来组成一个新列表，.append() 方法用于在列表末尾添加新的对象
            contour1 = np.array(contour1) #将组成的新列表转化为矩阵，方便下一步处理
            (x, y, w, h) = cv2.boundingRect(contour1) #用一个最小的矩形，把找到的所有的轮廓包起来，返回轮值x，y是矩阵左上点的坐标，w，h是矩阵的宽和高
        # cv2.rectangle(img_thresh, (x, y), (x + w, y+h), (255, 255, 255), 3)
        # cv2.imshow("show",img_thresh)
        if (len(contours) > 0):   # 如果检测到的轮廓数量大于0
            if w>img_h/3  and con_num > 1 : 
                mask=np.zeros_like(gray_img) 
                #将mask的部分进行白色填充，
                # 参数为填充区域的左上角顶将gray_img转化为全是0的矩阵并赋值给mask即全黑点和右下角顶点
                cv2.rectangle(mask, (x+w//2, y), (x + w, y + h-3), (255, 255, 255), cv2.FILLED) 

                #将优化后的二值图img_thresh中的mask区域提取出来给temp_img
                temp_img=cv2.bitwise_and(img_thresh, img_thresh, mask=mask)

                #对只剩下中间线的二值图图像进行轮廓检测   （希望是中间线）
                contours1, hierarchy = cv2.findContours(temp_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if (len(contours1) > 0):
                    contour2 = []
                    for c1 in range(len(contours1)):
                        for c2 in range(len(contours1[c1])):
                            contour2.append(contours1[c1][c2])
                    contour2 = np.array(contour2) #将中间线的轮廓信息存于contour2矩阵中
                    (x1, y1, w1, h1) = cv2.boundingRect(contour2) #中间线的轮廓信息
                    if con_num > 2 : #左边绿线时会看不清，断成两节
                        cv2.rectangle(Img_2, (x1, y1), (x1 + w1, img_h), (255, 255, 255), 3)#白框-----同时检测到绿线和绿白线给蓝白线画白框——永远贴着底画矩形框
                        pianyi=((x1+w1/2)-(img_h/2))*FOV_w/img_h #pianyi值为矩形方框的中线距离视野中央的实际距离
                        print(1)
                    else : #这个时候才是真正的蓝线
                        cv2.rectangle(Img_2, (x, y), (x + w, y + h), (255, 0, 0), 3) #蓝框-----------------只检测到蓝线并用蓝框画出
                        pianyi = 0+((x + w / 2) - (img_h / 2)) * FOV_w / img_h #80凑数,为了让车不开出赛道去
                        print(2,x,y,w,h)
            elif w<img_h/3  :
                cv2.rectangle(Img_2, (x, y), (x + w, y + h), (0, 255, 255), 3) #黄框----------只检测到绿白线并用黄框画出
                # 如果是绿白线
                if con_num>1: #轮廓数量大于1，就是有好几段绿色
                    pianyi = ((x + w / 2) - (img_h / 2)) * FOV_w / img_h #pianyi值为矩形方框的中线距离视野中央的实际距离
                    print(3)
                else: 
                    #  小绿白线
                    if h < 50 :
                        pianyi = ((x + w / 2) - (img_h / 2)) * FOV_w / img_h #pianyi值为矩形方框的中线距离视野中央的实际距离
                        print(4)
                    else:
                        pianyi = 0+((x + w / 2) - (img_h / 2)) * FOV_w / img_h
                        pianyi_text='right'
                        print(5)
            elif con_num == 1: #横向绿线和最后一小段绿白线的绿线
                if h < 50 :
                    cv2.rectangle(Img_2, (x, y), (x + w, y + h), (0, 255, 255), 3) #黄框----------------检测到了最后一小段的蓝白线的蓝色
                    pianyi = ((x + w / 2) - (img_h / 2)) * FOV_w / img_h
                    print(6)
                else:
                    cv2.rectangle(Img_2, (x, y), (x + w, y + h), (255, 0, 0), 3) #蓝框-----------------只检测到蓝线并用蓝框画出
                    pianyi = -83+((x + w / 2) - (img_h / 2)) * FOV_w / img_h #平滑过渡
                    print(7)
            else:
                print('nothing')
        cv2.imshow('show',Img_2)
        key=cv2.waitKey(5)
        if key==27:
            break
        cropped_img_1 = color_seperate_1(cropped_img_1)





