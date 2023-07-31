#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-
"""
弯道
红绿灯
文字识别
"""
import cv2
import queue
import rospy
import sys
import os
import signal
import time
import numpy as np
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Image
import std_msgs.msg as std_msgs
from dynamic_reconfigure.server import Server
from qz_vision.cfg import detector_dynamicConfig
from geometry_msgs.msg import PoseWithCovarianceStamped
# path = os.path.abspath(".")
sys.path.insert(0,"/home/cquer/2023_qingzhou/src/qz_vision/scripts")
from yolo_detect.detect import Detect
from swan import *
from traffic import *
from openvideo import gstreamer_pipeline



# 调试用
tiaoshi = False
tiaoshi_swan = False
traffic_fangan_choose = 1
swan_suanfa_choose = 2

cam = None

def quit():
    global cam
    try:
        rospy.loginfo("----------------------vedio released successfully-----------------------------[2]")
        cam.release()
        sys.exit()
    finally:
        rospy.loginfo("----------------------vedio released successfully-----------------------------[3]")
        sys.exit()

signal.signal(signal.SIGINT,quit)
signal.signal(signal.SIGTERM,quit)

Queue=queue.LifoQueue()
def LoacateCB(msg):
    if msg.data == 4:
        Queue.put("line")
    elif msg.data == 1:
        Queue.put("traffic")
    elif msg.data == 2:
        global thread_notGreen
        thread_notGreen = 80000
    return 0

# def OdomCallBack(msg):
#     return 0

if __name__ == "__main__":
    # global tiaoshi,tiaoshi_swan
    rospy.init_node("qz_vision")
    if tiaoshi:
        server = Server(detector_dynamicConfig,cb)

    locate_sub=rospy.Subscriber("/qingzhou_locate",std_msgs.Int32,LoacateCB)
    # odom_sub = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,OdomCallBack)

    # daoche_pub = rospy.Publisher("/dwa_flag",std_msgs.Int32,queue_size=1)
    move_pub=rospy.Publisher("/qz_cmd_vel_vision",AckermannDrive,queue_size=1)
    park_id_=rospy.Publisher("/park_flag",std_msgs.Int32,queue_size=1)
    traffic_flag = rospy.Publisher("/traffic_flag",std_msgs.Int32,queue_size=1)
    locate_pub=rospy.Publisher("/qingzhou_locate",std_msgs.Int32,queue_size=1)

    RoadLine_Pub = rospy.Publisher("/RoadLinePub",Image,queue_size=5)
    Traffic_Pub = rospy.Publisher("/Traffic_Pub",Image,queue_size=5)
    Img_Pub = rospy.Publisher("/Img",Image,queue_size=5)

    # 出弯道计数
    outcheck = 0
    # 丢弃前几张图片
    keep_count = 0
    # 发布的运动控制
    cmd = AckermannDrive()
    # swan参数
    if swan_suanfa_choose == 0:
        Kp,Kd = 1.4,0.0
        now_err, last_err = 0, 0
        D_error=[0, 0, 0]
        acceleration  = 0.8
        speed = 0 
        max_speed = 0.8
    elif swan_suanfa_choose == 1:
        Kp,Kd = 1.5,0.5    
        now_err, last_err = 0, 0
        D_error=[0, 0, 0]
        acceleration  = 0.5 #0.05
        speed = 0 
        max_speed = 0.5
    else:
        pianyi_hist = []
        acceleration  = 0.025
        speed = 0 
        max_speed = 1.2

    # 红绿灯
    greencount, notgreencount = 0, 0
    # 导入模型
    det_txt = Detect('text')
    det_traffic = Detect('traffic')
    start_time, end_time, delta_time = 0,0,0

    cam=cv2.VideoCapture(gstreamer_pipeline(flip_method=0),cv2.CAP_GSTREAMER)
    try:
        print("[VISION]------------------摄像头开启----------------") 
        while not rospy.is_shutdown():
            queue_out = Queue.get()
            print("[VISION]-----------开启新一轮视觉--------------")
            # 清除缓存
            for i in range(10):
                ret = cam.grab()
            """
            计算角度  1.图片识别 2.PID  
            发布运动
            出弯判断
            控制权交接flag
            """
            if queue_out == "line":
                print("[VISION]-------弯道开始--------")
                while cam.isOpened():
                    ret, img = cam.read()
                    if ret:
                        # 裁去噪声 改变尺寸
                        img = cv2.resize(img,(640,480))
                        if swan_suanfa_choose == 0:
                            img = img[img.shape[0]*3//5:img.shape[0],:]
                        elif swan_suanfa_choose == 1:
                            # img = cv2.resize(img, (320, 240), interpolation=cv2.INTER_AREA)
                            # img = getROI(img, 0, 95)
                            # img = img[img.shape[0]-75:,:]
                            img = img[img.shape[0]*3//4:,img.shape[1]//5:]
                        elif swan_suanfa_choose == 2:
                            img = img[img.shape[0]*3//5:,:]
                        if keep_count<10:
                            keep_count += 1    
                        if  keep_count==10:
                            # 计算角度  1.图片识别
                            if swan_suanfa_choose == 0:
                                expect_angle,show_img,outcheckflag = GreenInWhite(img)
                            elif swan_suanfa_choose == 1:
                                expect_angle,show_img,outcheckflag = CalculateShift(img)
                            elif swan_suanfa_choose == 2:
                                pianyi, show_img, outcheckflag = pianyi_detect(img,pianyi_hist)
                                # show_img = cv2.resize(show_img,dsize=None,fx=0.25,fy=0.25)
                            # 计算角度  2.PID 
                            if outcheckflag==1:
                                print("[VISION]----出圈了！！")
                                outcheck +=1
                            elif outcheckflag==0:
                                outcheck=0
                            # 地板是绿色时使用
                            # 控制权交接flag
                            if outcheck == 3:
                                cmd.speed = 0.0
                                cmd.steering_angle = 0.0
                                keep_count = 0
                                D_error  = [0,0,0]
                                expect_angle = 0
                                outcheck = 0
                                now_err, last_err = 0, 0
                                move_pub.publish(cmd)
                                # locate_pub.publish(5)
                                print("[VISION]----------弯道结束------------")
                                pianyi_hist = []
                                if not tiaoshi_swan:
                                    break                                                                                    
                            if swan_suanfa_choose != 2:
                                last_err = now_err
                                now_err = expect_angle
                                D_error = D_error[1:]
                                D_error.append(last_err - now_err)
                                cmd_angle = Kp * now_err + Kd * (D_error[2] * 0.65 + D_error[1] * 0.3 + D_error[0] * 0.05)
                                # 发布运动
                                speed += acceleration if speed <= max_speed else 0.0
                                cmd.speed = speed
                                cmd.steering_angle =   cmd_angle
                                move_pub.publish(cmd)
                                print("[VISION]--理想角度：%f --PID角度%f"%(expect_angle,cmd_angle))
                            else:
                                pianyi_hist.append(pianyi)
                                print(len(pianyi_hist))
                                twist = pianyi
                                if len(pianyi_hist)>=90:
                                    speed -= 8*acceleration if speed >= 0.5 else 0.0
                                else:
                                    speed = 0.8
                                cmd.speed = speed
                                cmd.steering_angle =   twist
                                print("[VISION]--角度：%f "%(twist))
                                move_pub.publish(cmd)
                            if tiaoshi_swan:
                                final_img_msg=Image()
                                final_img_msg.header=std_msgs.Header()
                                final_img_msg.width=show_img.shape[1]
                                final_img_msg.height=show_img.shape[0]
                                final_img_msg.encoding="bgr8"
                                final_img_msg.step=1920
                                final_img_msg.data=np.array(show_img).tostring()
                                RoadLine_Pub.publish(final_img_msg)
                            else:
                                Img = Image()
                                Img.header=std_msgs.Header()
                                Img.width=show_img.shape[1]
                                Img.height=show_img.shape[0]
                                Img.encoding="bgr8"
                                Img.step=1920
                                Img.data=np.array(show_img).tostring()
                                Img_Pub.publish(Img)
            # 红绿灯
            # 清除缓存
            # for i in range(25):
            #     ret = cam.grab()
            elif queue_out == "traffic":
                print("[VISION]------------红绿灯------------------")
                # 调试结束后删去
                # time.sleep(1)
                # traffic_flag.publish(1)
                # continue
                framecount = 0
                while cam.isOpened():

                    ret, img = cam.read()
                    if greencount == 5 :
                        traffic_flag.publish(1)
                        greencount = 0
                        notgreencount = 0
                        print("[VISION]---------------绿灯行-----------------")
                        # 调试时记得注释调
                        # if not tiaoshi:
                        #     break
                    elif notgreencount == 5:
                        traffic_flag.publish(0)
                        greencount = 0
                        notgreencount = 0
                        print("[VISION]---------------红灯停-----------------")
                    if ret:
                        if traffic_fangan_choose == 2:
                            if framecount <= 10:
                                framecount += 1
                                continue
                            else:
                                framecount = 0
                            result = det_traffic.detect(img)
                            if result == "":
                                continue
                            a,b,c,d = result["crop"]
                            if a==0 or b == 0 or c == 0 or d == 0:
                                print("[VISION]----0")
                            traffic_status = result["classes"]
                            if traffic_status == "red":
                                notgreencount += 1
                                print("red")
                            elif traffic_status == "green":
                                greencount += 1 
                                print("green")
                        if traffic_fangan_choose == 1:
                            img = cv2.resize(img, None, fx = 0.5, fy = 0.5, interpolation = cv2.INTER_CUBIC)
                            roi_img = shiftArea(img)
                            greenLight = getColorArea(color='Green',img=roi_img)
                            notgreenLight  = getColorArea(color='RandY',img = roi_img)
                            print("[VISION]------绿色",np.sum(greenLight))
                            print("[VISION]------红色",np.sum(notgreenLight))
                            if np.sum(notgreenLight) > thread_notGreen:
                                notgreencount += 1
                                show_img = notgreenLight
                            else:
                                if np.sum(greenLight)>thread:
                                    greencount+=1
                                    show_img = greenLight
                                else:
                                    print("[VISION]---No green")
                                    show_img = notgreenLight
                            Img = Image()
                            Img.header=std_msgs.Header()
                            Img.width=show_img.shape[1]
                            Img.height=show_img.shape[0]
                            Img.encoding="bgr8"
                            Img.step=1920
                            Img.data=np.array(show_img).tostring()
                            Img_Pub.publish(Img)
            # 文字识别
            if queue_out == "line":
                # 2是下面 1是上面  
                # park_id_.publish(2)
                # locate_pub.publish(11)
                # continue
                # 倒车注释掉，由dwa发
                hangtian,jidian,xinghang,sanyuan,cnt = 0,0,0,0,0
                ret,Img = cam.read()
                while cam.isOpened():
                    if ret :
                        cnt+=1
                        delta_time = end_time - start_time
                        print("[VISION]识别用时:%f"%delta_time)
                        end_time = time.time()
                        start_time = time.time()
                        result = det_txt.detect(Img)
                        if result == "":
                            cnt += 1
                        else:
                            park_id = result["classes"]
                            if park_id == 'hangtian':
                                hangtian+=1
                            elif park_id == 'sanyuan':
                                sanyuan+=1
                            elif park_id == 'jidian':
                                jidian+=1
                            elif park_id == 'xinghang':
                                xinghang+=1
                            if hangtian==1 or sanyuan ==1:
                                park_id_.publish(2)
                                print(park_id)
                                locate_pub.publish(11)
                                break
                            elif jidian == 1 or xinghang == 1:
                                print(park_id)
                                park_id_.publish(1)
                                locate_pub.publish(11)
                                break

                        if cnt == 10 :
                            park_id_.publish(2)
                            locate_pub.publish(11)
                            print('[VISON]-----------识别失败--------------')
                            break

    finally:
        rospy.loginfo("----------------------vedio released successfully-----------------------------[1]")
        cam.release()