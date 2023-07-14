#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

import cv2
import numpy as np


"""
参数接口
"""
Slowergreen = np.array([50, 40, 45])
Shighgreen = np.array([90, 255, 255])
# 白色HSV
lowerwhite=np.array([0, 0, 100])
upperwhite=np.array( [180, 20, 255])

angle_gain = 0.01
# angle_gain = 0.1

# 区赛出圈判断
OutThresh = 8000
"""
函数
"""
# 提取ROI
def getROI(img,x,y):
    h , w = img.shape[:2]
    poly = np.array([
         [(x, h - y), (w , h - y), (w, h), (x, h)]
    ])
    if len(img.shape) > 2:
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mask = np.zeros_like(gray_img)
    else:
        mask = np.zeros_like(img)
    cv2.fillPoly(mask,poly,255)
    masked_img = cv2.bitwise_and(img, img, mask = mask)
    return masked_img


# 提取绿色
def getGreenRoadLine(img):
	hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv_img, lowerb=Slowergreen, upperb=Shighgreen)
	color_img = cv2.bitwise_and(img, img, mask=mask)
	return color_img

# 提取白色
def getWhiteRoadLine(img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_img, lowerb=lowerwhite, upperb=upperwhite)
    whiteImg = cv2.bitwise_and(img, img, mask=mask)
    return whiteImg

# 获得蓝色车道线的边缘点,返回可以包含这个轮廓的最小矩形的特征
def getBlueRoadLine(img):
    blue_img = color_seperate_1(img)
    gray_blue = cv2.cvtColor(blue_img,cv2.COLOR_BGR2GRAY)
    ret,img_thresh = cv2.threshold(gray_blue,10,255,cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    img_thresh_blue = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
    img_thresh_blue = cv2.morphologyEx(img_thresh, cv2.MORPH_CLOSE, kernel)
    contours_blue, hierarchy = cv2.findContours(img_thresh_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if (len(contours_blue) > 0):
        contours_blue_2 = []
        for c1 in range(len(contours_blue)):
            for c2 in range(len(contours_blue[c1])):
                contours_blue_2.append(contours_blue[c1][c2])
        contours_blue_2 = np.array(contours_blue_2)
        (x2, y2, w2, h2) = cv2.boundingRect(contours_blue_2)
        return (x2,y2,w2,h2)
    else :
        return (0,0,0,0)
# Binary
def handleImg(img, low=10, kernel_size=4):
    gray_Img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, img_thresh = cv2.threshold(gray_Img, low, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_CLOSE, kernel)
    img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
    return img_thresh


# 计算便宜
def CalculateShift(img):
    green_img = getGreenRoadLine(img)
    binary = handleImg(green_img)
    # 列求和
    histogram = np.sum(binary[:, :], axis=0)
    midpoint = int(histogram.shape[0] / 2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint + 30:])
    if leftx_base != 0 and rightx_base != 0:
        base = rightx_base + midpoint
    else :
        base = leftx_base - 80 

    ROI_img = getROI(binary, base, 120)
    contours, hierarchy = cv2.findContours(ROI_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
		# 获取图心，即提取部分的图像的车道线中点坐标
        M = cv2.moments(contours[0])
        cx = int(M['m10']/(M['m00']+float("1e-5")))   # 加上float防止除数可能为零的情况
        cy = int(M['m01']/(M['m00']+float("1e-5")))
        cv2.line(img,(img.shape[1]//2,img.shape[0]),(cx,cy),(255,0,255),10)
        cv2.line(img,(base,0),(base,img.shape[0]),(255,0,0),10)
		# 防止中间有时候处理有问题导致找不到中点，同时在出弯时可以用来判断
        if cx == 0 and cy == 0:
            shift = 0
        else:
            shift = angle_gain *np.degrees(np.arctan((img.shape[1]//2 - cx)/(img.shape[0]-cy)))
    else:
        shift = 0
        # x_blue,y_blue,w_blue,h_blue = getBlueRoadLine(img)
        # if x_blue!=0:
        #     cv2.rectangle(img, (x_blue, y_blue), (x_blue + w_blue, h_blue), (255, 0, 255), 3) #红框
        #     shift = np.degrees(np.arctan(((x_blue+w_blue//2)-img.shape[1]//2)/(img.shape[0]//2)))
        # else:
    return shift,img

# 白色赛道里面的绿色中心线
def GreenInWhite(img):
    whiteRoadLine = getWhiteRoadLine(img)
    binary = handleImg(whiteRoadLine,kernel_size=12)
    conters, hierarchy=cv2.findContours(binary,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
    best_child = -1
    for i in range(len(conters)):
        M = cv2.moments(conters[i])
        # print(M['m00'])
        if M['m00'] > 80000 and hierarchy[0][i][2] != -1:
            first_child = hierarchy[0][i][2]
            best_child = first_child
            less_dist = 100000
            # traverse contours after the first child
            brother = first_child
            max_iter = 50
            cnt = 0
            while (brother != -1 and cnt <= max_iter):
                # print(cv2.moments(conters[brother])['m00'])
                if (cv2.moments(conters[brother])['m00'] > 600):
                    _,y,_,h = cv2.boundingRect(conters[brother])
                    if (y + h//2) <= less_dist:
                        less_dist = y + h//2
                        best_child = brother
                brother = hierarchy[0][brother][1]
                cnt += 1
                # print("In finding next children")
            # traverse contours before the first child
            brother = first_child
            max_iter = 50
            cnt = 0
            while (brother != -1 and cnt <= max_iter):
                if (cv2.moments(conters[brother])['m00'] > 600):
                    _,y,_,h = cv2.boundingRect(conters[brother])
                    if (y + h//2) <= less_dist:
                        less_dist = y + h//2
                        best_child = brother
                brother = hierarchy[0][brother][0]
                cnt += 1
                # print("In finding previous children")
            
            cv2.drawContours(img, conters, best_child, (255, 0, 0), thickness=20)
            break
    if best_child != -1:
        best_conter = conters[best_child]
        # if OutCheck(best_conter,img):
        M = cv2.moments(best_conter)
        cx = int(M['m10']/(M['m00']+float("1e-5")))   # 加上float防止除数可能为零的情况
        cy = int(M['m01']/(M['m00']+float("1e-5")))
        shift = angle_gain *np.degrees(np.arctan((img.shape[1]//2 - cx)/(img.shape[0]-cy)))
        # else:
        #     shift = 'shit'
    else:
        shift = 'shit'
    return shift,img

# 区赛判断出圈的函数
def OutCheck(conters,img):
    img = getGreenRoadLine(img)
    img = handleImg(img)
    mask = np.zeros_like(img)
    # print(conters.reshape(conters.shape[0],-1))
    # conters.reshape(conters.shape[0],-1)
    # cv2.fillPoly(mask,conters,255)
    x,y,w,h = cv2.boundingRect(conters)
    poly = np.array([
        [(x, y), (x+w, y), (x+w,y+ h), (x,y+ h)]  #将数组转化为矩阵，四个为长方形的四个顶点，从左上角开始顺时针,最下面竖着100个像素的图 #原来是90
    ])
    cv2.fillPoly(mask,poly,255)
    masked_img = cv2.bitwise_and(img,img,mask=mask)
    print(np.sum(masked_img))
    return (np.sum(masked_img)>=OutThresh)


# 出弯判断函数，寻找车道线 
def FindGreenBlockWhenYouWantKonwOutOfRoadLineOrNot(img):
    greenIimg = getGreenRoadLine(img)
    binary_img = handleImg(greenIimg)
    # x_blue,y_blue,w_blue,h_blue = getBlueRoadLine(img)
    print("[VISION]--------------出圈判断颜色阈值是：",np.sum(binary_img))
    # 区赛地板是绿的，大量绿色会出弯
    if np.sum(binary_img)>10000000:#50000
        return True,binary_img
    # elif x_blue and y_blue and w_blue and h_blue:
        # return True,binary_img
    else:
        return False,binary_img

FOV_w=105
flag = 0
pianyi_before = 0
cnt=0

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
    return (pianyi_now/70), show_img