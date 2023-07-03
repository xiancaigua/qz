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
import std_msgs.msg as std_msgs
from dynamic_reconfigure.server import Server
from qz_vision.cfg import detector_dynamicConfig
from geometry_msgs.msg import PoseWithCovarianceStamped
from yolo_detect.detect import Detect
import time

"""
S弯道用的参数
"""
# 识别标志位
recog_flag=False
dwa_flag = False
turn_on_recog_flag_flag=True
turn_on_dwa_flag_flag=True
lower1=np.array([0,90,46])
upper1=np.array([10,255,255])
lower2=np.array([156,43,46])
upper2=np.array([180,255,255])

# S弯蓝色块
lowerline=np.array([90, 43, 50])
upperline=np.array( [255, 255, 255])
# S弯红色块
lowerred2line = np.array( [0,43,255 ] )
upperred2line = np.array( [10,255,255 ] )
# S弯白色块
lowerwhite=np.array([0, 0, 100])
upperwhite=np.array( [180, 20, 255])
# S弯绿色块   # 81  46  60
Slowergreen = np.array([50,40,45])
Shighgreen = np.array([90,255,255])

threshofwhite = 100
kernel_size=8
roadlineflag=True # 判断是否还使用S弯道   
queue_out=None


"""
红绿灯用的参数
"""
break_flag=False
pub_flag=0
color_dict = {
    'Green':[[65, 80, 145], [77, 255, 255]],
    'RandY':[[0, 30, 50], [35, 255, 255]]
}
# color_dict = {
# 	'Green':[[40, 50, 145], [77, 255, 255]],
# 	'RandY':[[0, 30, 50], [35, 255, 255]]
# }
thread=-1
# thread=9000  #绿色像素值的限制，像素低于这个值会被筛选
#20000


"""
摄像头开启
"""
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




"""
动态参数解析函数
"""
def cb(config,level):
    global lowerline
    global upperline
    global thread
    global Slowergreen
    global Shighgreen
    global color_dict
    Slowergreen[0]=config.H1
    Slowergreen[1]=config.S1
    Slowergreen[2]=config.V1
    Shighgreen[0]=config.H2
    Shighgreen[1]=config.S2
    Shighgreen[2]=config.V2
    print(Slowergreen)
    print(Shighgreen)

    # color_dict["Green"][0][0]=config.H1
    # color_dict["Green"][0][1]=config.S1
    # color_dict["Green"][0][2]=config.V1
    # color_dict["Green"][1][0]=config.H2
    # color_dict["Green"][1][1]=config.S2
    # color_dict["Green"][1][2]=config.V2
    # thread = config.thread

    # print(lowerline)

    # global kernel_size
    # kernel_size=config.Kernel_size
    # global threshofwhite 
    # threshofwhite = config.ThreshOfWhite

    return config

def shiftArea(img):
    row, column, _ = img.shape
    temp_img = img[int(row*0.2):int(row), int(column*0.2):int(column*0.8)]
    return temp_img

# 根据传入的color，从color_dict中获取对应上下限提取对应的颜色
def getColorArea(img, color):
    hsv_img = cv2.cvtColor(temp_img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_img, lowerb=np.array(color_dict[color][0]), upperb=np.array(color_dict[color][1])) 
    color_img = cv2.bitwise_and(temp_img,temp_img, mask = mask)
    return color_img


"""
视觉处理使用的函数定义区——主要是弯道
"""
FOV_w=105
flag = 0
pianyi_before = 0
cnt=0

def pianyi_detect(img):
    """
    参数调试区
    """


    pianyi=0
    pianyi_text=''
    global pianyi_before
    global cnt
    ##############读取图像#########################
    (img_w, img_h) = img.shape[:2] #获取传入图片的长与宽   w是高 h是宽

    lane_img=img.copy() #复制一份获取的图像给lane_img
    cropped_img=region_of_interest(lane_img) #对图像进行ROI的分割
    # if cnt==0:
    #     cv2.imwrite('/home/cquer/2023_qingzhou/src/qz_vision/cropped_img.jpg',cropped_img)
    #     cnt+=1
    cropped_img_1=cropped_img.copy() #将ROI图像复制一份给cropped_img_1
    cropped_img = color_seperate(cropped_img) #将ROI图像中的绿色部分提取出来
    gray_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY) #将提取的  ROI的绿色部分转化为灰度图
    ####1.如果检测到绿色线或者绿白线，进行判断###########
    ret, img_thresh = cv2.threshold(gray_img,10, 255, cv2.THRESH_BINARY)  #大于10的地方就转化为白色255，返回两个值第一个是域值，第二个是二值图图像
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4, 4)) #返回一个4*4的椭圆形矩阵核，椭圆的地方是1，其他地方是0
    img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_CLOSE, kernel) #闭运算，运用核kernel先进行膨胀，再进行腐蚀
    img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel) #开运算，运用核kernel先进行腐蚀，再进行膨胀
    contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # print(len(contours))#findcontours返回两个值，一个是一组轮廓信息，还有一个是每条轮廓对应的属性
    #####顺带检测蓝色车道线###########
    cropped_img_1 = color_seperate_1(cropped_img_1)
    gray_img_blue = cv2.cvtColor(cropped_img_1, cv2.COLOR_BGR2GRAY)
    ret, img_thresh_blue = cv2.threshold(gray_img_blue, 10, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    img_thresh_blue = cv2.morphologyEx(img_thresh_blue, cv2.MORPH_OPEN, kernel)
    img_thresh_blue = cv2.morphologyEx(img_thresh_blue, cv2.MORPH_CLOSE, kernel)
    contours_blue, hierarchy = cv2.findContours(img_thresh_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if (len(contours_blue) > 0):
        contours_blue_2 = []
        for c1 in range(len(contours_blue)):
            for c2 in range(len(contours_blue[c1])):
                contours_blue_2.append(contours_blue[c1][c2])
        contours_blue_2 = np.array(contours_blue_2)
        (x2, y2, w2, h2) = cv2.boundingRect(contours_blue_2)
    nothing_point = 0 #无用的点
    if (len(contours) > 0):   # 如果检测到的轮廓数量大于0
        con_num = len(contours) #将轮廓的个数赋值给con_num
        contour1 = [] #将contour1 赋值为空列表，[]表示列表，列表是可变的序列
        for c1 in range(len(contours)): #遍历每一个轮廓
                for c2 in range(len(contours[c1])): #遍历每一个轮廓的轮廓上的点
                    contour1.append(contours[c1][c2]) #将每一个轮廓的每一个点都排列起来组成一个新列表，.append() 方法用于在列表末尾添加新的对象
        contour1 = np.array(contour1) #将组成的新列表转化为矩阵，方便下一步处理
        (x, y, w, h) = cv2.boundingRect(contour1) #用一个最小的矩形，把找到的所有的轮廓包起来，返回轮值x，y是矩阵左上点的坐标，w，h是矩阵的宽和高
        # ####################1.1 同时检测到两条绿线，删选出中间线，计算位置########################
        if w>img_h/3  and con_num > 1 : #如果整体轮廓的宽度大于三分之图片的宽度，则说明同时检测到了两条绿线

            mask=np.zeros_like(gray_img) 
            #将mask的部分进行白色填充，
            # 参数为填充区域的左上角顶将gray_img转化为全是0的矩阵并赋值给mask即全黑点和右下角顶点
            cv2.rectangle(mask, (x+(3*w)//4, y), (x + w, y + h-3), (255, 255, 255), cv2.FILLED) 

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
                    cv2.rectangle(img, (x1, y1), (x1 + w1, img_h), (255, 255, 255), 3)#白框-----同时检测到绿线和绿白线给蓝白线画白框——永远贴着底画矩形框
                    pianyi=((x1+w1/2)-(img_h/2))*FOV_w/img_h #pianyi值为矩形方框的中线距离视野中央的实际距离
                    if pianyi>0:
                        pianyi_text='right'
                    elif pianyi<0:
                        pianyi_text='left'
                        # pianyi+=80
                    else:
                        pianyi_text = 'stright'
                    # 防止车进弯道时过于靠近绿线
                    if  len(contours_blue)>0 and np.sum(img_thresh_blue)<10000 and  pianyi_text == 'right':
                        pianyi = pianyi // 3
                        print('防止车进弯道时过于靠近绿线')
                        # print(np.sum(img_thresh_blue))
                    print(1,pianyi_text)
                else : #这个时候才是真正的绿线
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3) #蓝框-----------------只检测到蓝线并用蓝框画出
                    pianyi = -83+((x + w / 2) - (img_h / 2)) * FOV_w / img_h #80凑数,为了让车不开出赛道去
                    pianyi_text='right'
                    print(2,pianyi_text)
        #########################1.2 只检测到一条线，需要判断是绿白线还是绿线##############
        elif w<img_h/3  :
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 3) #黄框----------只检测到绿白线并用黄框画出
            # 如果是绿白线
            if con_num>1: #轮廓数量大于1，就是有好几段绿色,但是会出现边线误识别成中线，因为线可能会断掉？
                pianyi = ((x + w / 2) - (img_h / 2)) * FOV_w / img_h #pianyi值为矩形方框的中线距离视野中央的实际距离
                if pianyi > 0:
                    #print('右偏')
                    pianyi_text='right'
                elif pianyi<0:
                    #print('左偏')
                    pianyi_text='left'
                    pianyi -= 80
                else:
                    # print('左偏')
                    pianyi_text = 'stright'
                print(3,pianyi_text)
            # 再分类
            else: 
                #  小绿白线
                if h < 50 and len(contours_blue)>0 and x>50:
                    pianyi = ((x + w / 2) - (img_h / 2)) * FOV_w / img_h #pianyi值为矩形方框的中线距离视野中央的实际距离
                    print(4,pianyi_text)
                    if pianyi > 0:
                        pianyi_text='right'
                    elif pianyi<0:
                        pianyi_text='left'
                    else:
                        pianyi_text = 'stright'            
                # 真边绿线,之看见左边的绿色线
                else:
                    pianyi = -83+((x + w / 2) - (img_h / 2)) * FOV_w / img_h
                    pianyi_text='right'
                    print(5,pianyi_text)
        elif con_num == 1: #横向绿线和最后一小段绿白线的绿线
                if h < 50 and len(contours_blue)>0 and x>50:
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 3) #黄框----------------检测到了最后一小段的蓝白线的蓝色
                    pianyi = ((x + w / 2) - (img_h / 2)) * FOV_w / img_h
                    print(7,pianyi_text)
                    if pianyi > 0:
                        #print('右偏')
                        pianyi_text='right'
                    elif pianyi<0:  
                        #print('左偏')
                        pianyi_text='left'
                    else:
                        # print('左偏')
                        pianyi_text = 'stright'
                else: #看见一块绿色并且真的是绿线
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3) #蓝框-----------------只检测到绿线并用蓝框画出
                    pianyi = -100+((x + w / 2) - (img_h / 2)) * FOV_w / img_h #平滑过渡
                    pianyi_text='right'
                    print(8,pianyi_text)
        else : #检测到了左下角的点了
            nothing_point = 1


    # 2.未检测到绿线或者绿白线，就检测蓝线
    else:
        if len(contours_blue)>0:
            if (x2+w2/2)>(2*img_h)/3:
                cv2.rectangle(img, (x2, y2), (x2 + w2, img_h), (255, 0, 255), 3) #红框
                if h2 < 70:
                    pianyi = pianyi_before
                    print(9)
                else :
                    pianyi = 200 - ((x2 + w2 / 2) - (img_h / 2)) * FOV_w / img_h
                    pianyi_text='left'
                    print(10,pianyi_text)

    show_img = img

    pianyi_now = abs(pianyi)

    angle_gain_left = 80 # 这个值随速度变化，速度0.3差不多对应40的angle_gain
    angle_gain_right = 80
    if pianyi_text == 'right' :
        pianyi_now = 0 - pianyi_now -  angle_gain_right

    elif  pianyi_text == 'left' :
        pianyi_now =  pianyi_now + angle_gain_left

    #  
    if pianyi_before == -pianyi_now or nothing_point ==1 : #这一句如果加上防止突变有点危险
        pianyi_now = pianyi_before       
        print("*****检测到干扰*******")   
    pianyi_before = pianyi_now
    # print('-------------------',pianyi_now )
    return pianyi_now, show_img




# #####################提取ROI#############################
def region_of_interest(r_image):
    h = r_image.shape[0] #图片的高
    w = r_image.shape[1] #图片的宽
    ImgH_=180  #90
    gray = cv2.cvtColor(r_image, cv2.COLOR_BGR2GRAY) #转化为灰度图

    poly = np.array([
        [(0, h-ImgH_), (w, h-ImgH_), (w, h), (0, h)]  #将数组转化为矩阵，四个为长方形的四个顶点，从左上角开始顺时针,最下面竖着100个像素的图 #原来是90
    ])
    mask = np.zeros_like(gray) #输入为矩阵gray，输出为形状和gray一致的矩阵，其元素全部为黑色0
    cv2.fillPoly(mask, poly, 255)   #将mask的poly部分填充为白色255
    masked_image = cv2.bitwise_and(r_image,r_image, mask=mask)  #将r_image的mask区域提取出来给masked_image
    return masked_image

# ###############################green分割############################
def color_seperate(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   #对目标图像进行色彩空间转换
    lower_hsv = Slowergreen          #设定green下限
    upper_hsv = Shighgreen        #设定green上限
    mask = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)  #依据设定的上下限对目标图像进行二值化转换，低于lower,高于upper都变成0，在中间为255
    dst = cv2.bitwise_and(image, image, mask=mask)    #将image的mask区域提取出来给dst,即找到蓝色区域并赋值给dst
    return dst


# ############################blue分割##############################
def color_seperate_1(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   #对目标图像进行色彩空间转换
    lower_hsv = np.array([100, 43, 46])          
    upper_hsv = np.array([160, 255, 255])        
    mask = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)  #依据设定的上下限对目标图像进行二值化转换，低于lower,高于upper都变成0，在中间为255
    dst = cv2.bitwise_and(image, image, mask=mask)    #将image的mask区域提取出来给dst,即找到红色区域并赋值给dst
    # cv2.imshow('red', dst)  #查看红色区域的图像
    # # cv2.waitKey(3)
    return dst



# 废弃
def getRoadLineThroughHSV(img):
    hsv_img=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    mask1=cv2.inRange(hsv_img,lowerwhite,upperwhite)
    # mask2=cv2.inRange(hsv_img,lowerwhite,upperwhite)

    masked_img1=cv2.bitwise_and(img,img,mask=mask1)
    # masked_img2=cv2.bitwise_and(img,img,mask=mask2)
    # masked_img=masked_img1+masked_img2
    masked_img=masked_img1

    gray_img=cv2.cvtColor(masked_img,cv2.COLOR_BGR2GRAY)
    ret,binary_img=cv2.threshold(gray_img,10,255,cv2.THRESH_BINARY)
    # kernel1=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
    # kernel2=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2,2))
    # binary_img=cv2.morphologyEx(binary_img,cv2.MORPH_OPEN,kernel2)
    # binary_img=cv2.morphologyEx(binary_img,cv2.MORPH_CLOSE,kernel1)

    return binary_img

# 废弃
def getLineCenter(binary_img):
    shift=0
    cx=0
    lastcx=0
    cy=0
    lastcy=0
    count=0
    
    # histogram=np.sum(binary_img[:,:],axis=0)
    # midpoint = int(histogram.shape[0] / 2)
    # leftx_base = np.argmax(histogram[:midpoint])
    # rightx_base = np.argmax(histogram[midpoint + 30:])
    # if leftx_base != 0 and rightx_base != 0:
    # 	base = leftx_base + 30
    # else:
    # 	base = rightx_base + midpoint + 30
    
    # histogram2=np.sum(binary_img[:,:],axis=1)
    # midpoint2 = int(histogram2.shape[0] / 2)
    # lefty_base = np.argmax(histogram2[:midpoint2])
    # righty_base = np.argmax(histogram2[midpoint2 + 30:])
    # if lefty_base != 0 and righty_base != 0:
    # 	base2 = lefty_base + 30
    # else:
    # 	base2 = righty_base + midpoint2 + 30




    final_img=binary_img
    # final_img=binary_img[:img.shape[0]-base2,:base]
    contours=cv2.findContours(final_img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    contours=contours[0]
    if len(contours)>0:
        M = cv2.moments(contours[0])
        lastcx=cx
        lastcy=cy
        cx = int(M['m10']/(M['m00']+float("1e-5")))   # 加上float防止除数可能为零的情况
        cy = int(M['m01']/(M['m00']+float("1e-5")))
        # 防止抽搐
        if count>=5:
            if abs(cx-lastcx)>binary_img.shape[1]//2:
                cx=lastcx
            if abs(cy-lastcy)>binary_img.shape[0]//5:
                cy=lastcy
        count+=1
        # cv2.line(final_img,(cx,cy),(final_img.shape[1]//2,final_img.shape[0]),(0,0,0),thickness=5)
        # 防止中间有时候处理有问题导致找不到中点，同时在出弯时可以用来判断
        if cx == 0 and cy == 0:
            shift = 0
        else:
            shift = np.degrees(np.arctan(float(50+binary_img.shape[1]/2 - cx)/float( binary_img.shape[0]-cy)))/(-57.3)
    # cv2.imwrite("/home/cquer/2023_qingzhou/src/qz_vision/final.jpg",final_img)
    return shift,final_img
# 提取白色车道县
def getWhiteLine(img):
    gray_img=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ret,binary_img=cv2.threshold(gray_img,threshofwhite,255,cv2.THRESH_BINARY)
    kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(kernel_size,kernel_size))
    binary_img=cv2.morphologyEx(binary_img,cv2.MORPH_OPEN,kernel)
    binary_img=cv2.morphologyEx(binary_img,cv2.MORPH_CLOSE,kernel)
    return binary_img
# 废弃
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

# 王珂提出的寻找内空洞并且计算图心的算法,采用
def FindBlueCenterLine(binary_img,img,defalt_pos):
    delta_x=binary_img.shape[1]//6
    # position=defalt_pos

    conters=cv2.findContours(binary_img,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[0]
    Appropriate_M=[]
    last_area=0
    if len(conters)>0:
        for c in conters:
            M=cv2.moments(c)
            if M['m00']>10:
                if M['m00']<10000:
                    if M['m00']>last_area:
                        last_area=M['m00']
                        img=cv2.drawContours(img,c,-1,(0,0,255),3)
                        Appropriate_M.append((int(M['m10']/(M['m00']+float("1e-5"))),int(M['m01']/(M['m00']+float("1e-5")))))
    if len(Appropriate_M)>0:
        pt = Appropriate_M[-1]
        if delta_x>abs(pt[0]-defalt_pos[0]):
            position=pt
            # delta_x=abs(pt[0]-binary_img.shape[1]//2)
        # for pt in Appropriate_M:
        else:
            position = ((1*pt[0])//16+(15*defalt_pos[0])//16,(1*pt[1])//16+(15*defalt_pos[1])//16)
    else:
        position=defalt_pos
    
    cv2.line(img,position,(img.shape[1]//2,img.shape[0]),(255,0,0),thickness=5)

    x=position[0]
    y=position[1]
    shift = np.degrees(np.arctan(float(binary_img.shape[1]/2 - x)/float(float("1e-5") + binary_img.shape[0]-y)))/(-57.3)
    # vel=(binary_img.shape[0]-y)*0.001+0.05
    vel=0.3

    return img,position,shift,vel

# 出弯判断函数，寻找车道线 
def FindBlueBlockWhenYouWantKonwOutOfRoadLineOrNot(img):
    lower_hsv = np.array([100, 43, 46])          
    upper_hsv = np.array([160, 255, 255]) 
    hsv_img=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    mask=cv2.inRange(hsv_img,Slowergreen,Shighgreen)
    masked_img=cv2.bitwise_and(img,img,mask=mask)

    mask_blue = cv2.inRange(hsv_img,lower_hsv,upper_hsv)
    mask_blue_img = cv2.bitwise_and(img,img,mask=mask_blue)

    gray_img=cv2.cvtColor(masked_img+mask_blue_img,cv2.COLOR_BGR2GRAY)
    _,binary_img=cv2.threshold(gray_img,50,255,cv2.THRESH_BINARY)
    kernel1=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2,2))
    kernel2=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2,2))
    binary_img=cv2.morphologyEx(binary_img,cv2.MORPH_OPEN,kernel2)
    binary_img=cv2.morphologyEx(binary_img,cv2.MORPH_CLOSE,kernel1)
    # print(np.sum(binary_img))
    if np.sum(binary_img)>1200000:
        return True,binary_img
    else:
        return False,binary_img

"""
文字识别区
"""
def do_recog():
    return 'p1'


"""
回调函数
"""
def OdomCallBack(msg):
    global	pub_flag,recog_flag,dwa_flag,turn_on_recog_flag_flag,turn_on_dwa_flag_flag
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    # 红绿灯的一个复位标志位
    if y<-4.8:
        pub_flag=1
        turn_on_recog_flag_flag=True
        turn_on_dwa_flag_flag=True
    # 开启识别吗？
    # 到达位置且未发送停车区FLAG才允许置位
    #-1.4   -2.0
    if x<-1.0 and y>-1.0 and turn_on_recog_flag_flag:
        recog_flag=True
    # 开启DWA吗
    # DWA的FLAG只会在S弯结束复位时复位，此后不再发布消息
    if x<-1.0 and y>-1.0 and roadlineflag and turn_on_dwa_flag_flag:
        dwa_flag=True
        print("---------DWA 启动！[qz_vision]-----------------------")
    return 0

Queue=queue.LifoQueue()
def LoacateCB(msg):
    global roadlineflag
    global	break_flag
    global queue_out
    if msg.data==4:
        roadlineflag=True
        Queue.put('line')
        # queue_out='line'
    elif msg.data == 1:
        break_flag=False
        Queue.put('traffic')
        # queue_out='traffic'

    

if __name__=="__main__":
    rospy.init_node("qz_vision")

    """
    动态参数
    """
    # server = Server(detector_dynamicConfig,cb)

    """
    ROS接口区
    """
    # detect()
    # qz_cmd_vel_vision  调试完话题换成这个
    move_pub=rospy.Publisher("/qz_cmd_vel_vision",Twist,queue_size=1)
    
    locate_sub=rospy.Subscriber("/qingzhou_locate",std_msgs.Int32,LoacateCB)
    # locate_pub=rospy.Publisher("/qingzhou_locate",std_msgs.Int32,queue_size=1)
    traffic_flag = rospy.Publisher("/traffic_flag",std_msgs.Int32,queue_size=1)
    park_id_=rospy.Publisher("/Park_id",std_msgs.Int32,queue_size=1)
    dwa_flag_=rospy.Publisher("/dwa_flag",std_msgs.Int32,queue_size=1)
    filter_dwa_flag_=rospy.Publisher("/filter_dwa_flag",std_msgs.Int32,queue_size=1)
    odom_sub = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,OdomCallBack)
    # 显示用发布者
    traffic_pub=rospy.Publisher("/TrafficImg",Image,queue_size=5)
    bianry_pub=rospy.Publisher("/BinaryImg",Image,queue_size=5)
    RoadLine_Pub = rospy.Publisher("/RoadLinePub",Image,queue_size=5)
    raw_pub = rospy.Publisher("/RawImg",Image,queue_size=5)


    """
    参数区
    """
    
    #S弯道使用的参数 
    cou=0
    Kp, Kd, Ki = 2.0, 0.1,0.5
    D_shift=[0, 0, 0]
    angle=0
    now_shift=0
    last_shift =  0
    outcheck = 0


    #红绿灯使用的参数
    green_count=0
    notgreen_count=0	
    color_flag=0 
    dict=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    parameters =  cv2.aruco.DetectorParameters_create()
    start_time =0
    end_time =0
    
    # pianyibefore = 0
    wheelbase = 0.31
    pianyicount = 0
    pianyisamelist = [0,0,0,0,0,0,0]#大概需要4/40=0.1s判断车车有没有出去，可能太短了
    controlFlag = 10 #原来是10
    openColorDetector = 0 #原来是0
    global flag_traffic
    flag_traffic = 0
    redtime = 0
    acc = -0.1
    v = 0.5
    cmdData = Twist()


    det = Detect()
    cam=cv2.VideoCapture(gstreamer_pipeline(flip_method=0),cv2.CAP_GSTREAMER) 
    # 程序会在这里阻塞 调试视觉时关掉，之后记得开启
    print("欧尼酱，视觉开始了哟～","FBI open the camera!------------------")
    while not rospy.is_shutdown():
        queue_out=Queue.get()   
        if (queue_out=="line" and roadlineflag):
            print("欧尼酱,S弯开始了~")
            filter_dwa_flag_.publish(0)
            keep_count = 0  # 前50张图片忽略不计
            twist = 0
            twist_before = 0.026
            twist_count = 0
            while cam.isOpened():
                delta_time = end_time-start_time
                start_time = time.time()
                print("程序用时：%.4f"%delta_time)
                # 出弯判断
                # *******************重点：切换状态机至DWA倒车*******************
                # S弯参数复位
                # 切换状态至TOSTART
                # 给出开启DWA导航的FLAG_DWA，在定位函数中进行
                # 由定位开启文字识别的子程序，并根据识别结果发布停车位置的FLAG_POS
                if outcheck == 10:
                    # 复位
                    outcheck = 0
                    roadlineflag=False
                    dwa_flag=False
                    recog_flag = False
                    # 切换状态
                    # locate_pub.publish(5)
                    filter_dwa_flag_.publish(1)
                    print("欧尼酱，S弯结束了")
                    break
                ret, Img = cam.read()
                # 出弯小判断，提取赛道，确定pix值区间，低于某个值则认为出圈
                if ret:
                    if keep_count>=50:
                            FindLine, FindBlueBlockWhenYouWantKonwOutOfRoadLineOrNot_img = FindBlueBlockWhenYouWantKonwOutOfRoadLineOrNot(Img[Img.shape[0]//2+50:,:])                            
                            if FindLine:
                                outcheck = 0
                            else:
                                outcheck = 1
                    else:
                        # 前50张图片忽略不计
                        keep_count+=1
                # 计算弯道转角以及发布可视化&运控  recog_flag and
                if  keep_count==70:
                    park_id = det.detect(Img)
                    print(park_id)
                    keep_count=50
                    if park_id=='hangtian' or park_id=='jidian':
                        park_id_.publish(1)
                        recog_flag=False
                        turn_on_recog_flag_flag=False
                    elif park_id=='xinghang' or park_id=='sanyuan':
                        park_id_.publish(2)
                        recog_flag=False
                        turn_on_recog_flag_flag=False
                else:
                    keep_count+=1
                    # 识别失败，放弃倒车
                if ret and keep_count>=50:
                    Img_2 = cv2.resize(Img,(640,480))
                    Img_2 = Img_2[3: 475,3:635]  #切割掉左右下角干扰点
                    Img_2 = cv2.resize(Img_2,(640,480))
                    pianyi, show_img = pianyi_detect(Img_2)
                    twist = pianyi/7000
                    # twist *= wheelbase/v
                    # 忽略小角度变化, 平滑轨迹
                    if abs(twist-twist_before) < 0.001:
                        twist = twist_before

                    if outcheck==10:
                        cmdData.angular.z=0
                        cmdData.linear.x = 0
                    else:
                        # cmdData.angular.z=0
                        # cmdData.linear.x = 0
                        cmdData.angular.z=twist
                        # print(cmdData.angular.z)
                        cmdData.linear.x = v
                    move_pub.publish(cmdData)
                    pianyi_before = pianyi
                    twist_before = twist
                    for index,value in enumerate(pianyisamelist):
                        if(not(index == len(pianyisamelist)-1)):#每个元素往前移动
                            pianyisamelist[index] = pianyisamelist[index+1]
                        else:#列表最后一个进来
                            pianyisamelist[index] = pianyi
                    if(len(set(pianyisamelist)) == 2):
                            pianyi = 999
                    final_img_msg=Image()
                    final_img_msg.header=std_msgs.Header()
                    final_img_msg.width=show_img.shape[1]
                    final_img_msg.height=show_img.shape[0]
                    final_img_msg.encoding="bgr8"
                    final_img_msg.step=1920
                    final_img_msg.data=np.array(show_img).tostring()
                    RoadLine_Pub.publish(final_img_msg)
                # 由定位开启文字识别的子程序，并根据识别结果发布停车位置的FLAG_POS
                # 给出开启DWA导航的FLAG_DWA，在定位函数中进行
                if dwa_flag:
                    dwa_flag_.publish(1)
                    turn_on_dwa_flag_flag=False
                    dwa_flag=False
                end_time = time.time()
                


                # binary2_msg=Image()
                # binary2_msg.header=std_msgs.Header()
                # binary2_msg.width=binary_img.shape[1]
                # binary2_msg.height=binary_img.shape[0]
                # binary2_msg.encoding="mono8"
                # binary2_msg.step=1920
                # binary2_msg.data=np.array(binary_img).tostring()
                # raw_pub.publish(binary2_msg)


        # not break_flag ：break_flag用于判断是否在执行😡绿灯任务的标志位，在LOAD会变成FALSE，允许执行
        elif queue_out=="traffic" and (not break_flag):
            print("[INFO]---I am in the traffic now!!!")
            while cam.isOpened():
                # 这一圈的红绿灯检测任务彻底结束
                # 检测到了绿灯，发布状态机到TOUNLOAD，复位
                if color_flag==1:
                        print("Green, you can go.")
                        # locate_pub.publish(8)
                        traffic_flag.publish(1)
                        break_flag=True
                        color_flag, green_count, notgreen_count = 0, 0, 0
                        break
                # 未检测到绿灯，复位，保持状态机在LOAD
                elif color_flag==2:
                        print("Red or yellow, stop.")
                        color_flag, green_count, notgreen_count = 0, 0, 0
                ret,img = cam.read()
                # print(img)
                if ret:
                    img = cv2.resize(img, None, fx = 0.5, fy = 0.5, interpolation = cv2.INTER_CUBIC)
                    if True:
                        # 截取有用的区域
                        temp_img=shiftArea(img)
                        # 获取红色黄色的色图(bushi)和绿色的色图(bushi)
                        green_Limg = getColorArea(temp_img, 'Green')
                        RandY_Limg = getColorArea(temp_img, 'RandY')

                        # gray_green = cv2.cvtColor(green_Limg,cv2.COLOR_BGR2GRAY)
                        # _,gray_green = cv2.threshold(gray_green,thread,255,cv2.THRESH_BINARY)
                        
                        #由设置的阈值thread确定是否绿灯亮阈值 
                        print("I am counting green   ------------   ",np.sum(green_Limg))
                        if np.sum(green_Limg) > thread:
                            green_count += 1
                        # elif np.sum(RandY_Limg) > thread:
                        else:
                            # print("I am counting not green")
                            notgreen_count += 1
                        # 数八次如果都是绿灯就给绿灯的标志位
                        if green_count == 8:
                            color_flag = 1
                        elif notgreen_count == 8:
                            color_flag = 2

                        # traffic_msg=Image()
                        # header=std_msgs.Header()
                        # header.frame_id="Camera"
                        # traffic_msg.header=header
                        # traffic_msg.width=green_Limg.shape[1]
                        # traffic_msg.height=green_Limg.shape[0]
                        # traffic_msg.encoding="bgr8"
                        # traffic_msg.step=1920
                        # traffic_msg.data=np.array(green_Limg).tostring()
                        # traffic_pub.publish(traffic_msg)

                        # redmsg=Image()
                        # header=std_msgs.Header()
                        # header.frame_id="Camera"

                        # redmsg.header=header
                        # redmsg.width=RandY_Limg.shape[1]
                        # redmsg.height=RandY_Limg.shape[0]
                        # redmsg.encoding="bgr8"
                        # redmsg.step=1920
                        # redmsg.data=np.array(RandY_Limg).tostring()
                        # gray_pub.publish(redmsg)

                    # RVIZ显示便于调试
                    # imgmsg=Image()
                    # header=std_msgs.Header()
                    # header.frame_id="Camera"

                    # imgmsg.header=header
                    # imgmsg.width=img.shape[1]
                    # imgmsg.height=img.shape[0]
                    # imgmsg.encoding="bgr8"
                    # imgmsg.step=1920
                    # imgmsg.data=np.array(img).tostring()
                    # # print("i am in the while loop")
                    # traffic_pub.publish(imgmsg)

        """
        OLD的S弯道
        """

        # if ret:
        # 	Img=Img[Img.shape[0]//2+50:,:]
        # 	FindLine,blueblock = FindBlueBlockWhenYouWantKonwOutOfRoadLineOrNot(Img)

        # 	if FindLine:
        # 		outcheck = 0
        # 	else:
        # 		outcheck += 1
        # 	# cv2.imwrite("/home/cquer/2023_qingzhou/src/qz_vision/RAW.jpg",Img)

        # 	binary_img=getWhiteLine(Img)
        # 	if cou==0:
        # 		defalt_pos=(binary_img.shape[1]//2,binary_img.shape[0]//2)
        # 	cou+=1
        # 	roallineFinalImg,position,shift,vel=FindBlueCenterLine(binary_img,Img,defalt_pos)
        # 	defalt_pos=position
            
            
            
        # 	if shift:
        # 		last_shift=now_shift
        # 		now_shift=shift
        # 		D_shift.append(shift)

        # 		angle=Kp*now_shift+Kd*(now_shift-last_shift)+Ki*(sum(D_shift)/len(D_shift))
        # 	else:
        # 		angle=0
        # 	# print(angle)
        # 	if outcheck==6:
        # 		cmdData.linear.x=0
        # 		cmdData.angular.z=0
        # 		locate_pub.publish(5)

        # 	else:
        # 		cmdData.linear.x=vel
        # 		cmdData.angular.z=angle*12
        # 	move_pub.publish(cmdData)
                        

    cam.release()
