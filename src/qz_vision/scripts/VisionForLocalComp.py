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
swan_suanfa_choose = 'wangke'
traffic_fangan_choose = 2



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
    # global tiaoshi,tiaoshi_swan,swan_suanfa_choose
    rospy.init_node("qz_vision")
    if tiaoshi:
        server = Server(detector_dynamicConfig,cb)

    locate_sub=rospy.Subscriber("/qingzhou_locate",std_msgs.Int32,LoacateCB)
    # odom_sub = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,OdomCallBack)

    dwa_pub = rospy.Publisher("/dwa_flag",std_msgs.Int32,queue_size=1)
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
    # PID Kp, Kd, Ki = 1.0,0.35or 0.25,1.0    
    # Kp, Kd= 0.8,0.2        #速度0.5  0.8,0.5   
    Kp,Kd = 1.0,0.0
    now_err, last_err = 0, 0
    D_error=[0, 0, 0]
    acceleration  = 0.04
    speed = 0 

    # 红绿灯
    greencount, notgreencount = 0, 0
    # 导入模型
    det_txt = Detect('text')
    det_traffic = Detect('traffic')
    start_time, end_time, delta_time = 0,0,0
    cam=cv2.VideoCapture(gstreamer_pipeline(flip_method=0),cv2.CAP_GSTREAMER)
    print("[VISION]------------------摄像头开启----------------") 
    try:
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
                        img = img[img.shape[0]*3//5:img.shape[0],:]
                        if keep_count<10:
                            keep_count += 1    
                        if  keep_count==10:

                            if FindGreenBlockWhenYouWantKonwOutOfRoadLineOrNot(img)[0]:
                                print("[VISION]----出圈了！！")
                                outcheck +=1
                            # 控制权交接flag
                            if outcheck == 5:
                                cmd.speed = 0.0
                                cmd.steering_angle = 0.0
                                keep_count = 0
                                D_error  = [0,0,0]
                                expect_angle = 0
                                outcheck = 0
                                now_err, last_err = 0, 0
                                move_pub.publish(cmd)
                                # 倒车解注释
                                # dwa_pub.publish(1)
                                print("[VISION]----------弯道结束------------")
                                break
                            
                            # 计算角度  1.图片识别
                            expect_angle,show_img = GreenInWhite(img)
                            # 计算角度  2.PID 
                            if expect_angle == 'shit' :
                                # outcheck += 1
                                print("[VISION] ---There is a shit!-----")
                            else:
                                outcheck = 0
                                last_err = now_err
                                now_err = expect_angle
                                D_error = D_error[1:]
                                D_error.append(last_err - now_err)
                                cmd_angle = Kp * now_err + Kd * (D_error[2] * 0.8 + D_error[1] * 0.15 + D_error[0] * 0.05)
                                # cmd_angle = Kp * expect_angle + Kd*( last_err - now_err)
                                # 发布运动
                                speed += acceleration if speed <= 0.5 else 0.0
                                cmd.speed = speed
                                cmd.steering_angle =   cmd_angle
                                cmd.steering_angle_velocity = cmd_angle/(0.1)   #0.05/0.1  角度·缩小PID
                                # cmd.steering_angle_velocity = cmd_angle/(0.8)
                                move_pub.publish(cmd)
                                print("[VISION]--理想角度：%f --PID角度%f"%(expect_angle,cmd_angle))
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
                while cam.isOpened():

                    ret, img = cam.read()
                    if greencount == 3 :
                        traffic_flag.publish(1)
                        greencount = 0
                        notgreencount = 0
                        print("[VISION]---------------绿灯行-----------------")
                        # 调试时记得注释调
                        if not tiaoshi:
                            break
                    elif notgreencount == 3:
                        traffic_flag.publish(0)
                        greencount = 0
                        notgreencount = 0
                        print("[VISION]---------------红灯停-----------------")
                    if ret:
                        # img = cv2.resize(img, None, fx = 0.5, fy = 0.5, interpolation = cv2.INTER_CUBIC)
                        result = det_traffic.detect(img)
                        if result == "":
                            continue
                        a,b,c,d = result["crop"]
                        if a==0 or b == 0 or c == 0 or d == 0:
                            print("[VISION]----0")
                        traffic_status = result["classes"]
                        if traffic_fangan_choose == 2:
                            if traffic_status == "red":
                                notgreencount += 1
                                print("red")
                            elif traffic_status == "green":
                                greencount += 1 
                                print("green")
                        if traffic_fangan_choose == 1:
                            greenLight = getColorArea(color='Green',img=roi_img)
                            notgreenLight  = getColorArea(color='RandY',img = roi_img)
                            print("Green",np.sum(greenLight))
                            if np.sum(notgreenLight) > thread_notGreen:
                                notgreencount += 1
                            else:
                                if np.sum(greenLight)>thread:
                                    greencount+=1
                                else:
                                    print("[VISION]---No green")

            # 文字识别
            if queue_out == "line":
                # 倒车注释掉，由dwa发
                locate_pub.publish(5)
                hangtian,jidian,xinghang,sanyuan,cnt = 0,0,0,0,0
                ret,Img = cam.read()
                while cam.isOpened():
                    if ret :
                        cnt+=1
                        delta_time = end_time - start_time
                        print("[VISION]识别用时:%f"%delta_time)
                        end_time = time.time()
                        start_time = time.time()
                        park_id = det_txt.detect(Img)["classes"]
                        if park_id == 'hangtian':
                            hangtian+=1
                        elif park_id == 'sanyuan':
                            sanyuan+=1
                        elif park_id == 'jidian':
                            jidian+=1
                        elif park_id == 'xinghang':
                            xinghang+=1
                        if hangtian==1 or sanyuan ==1:
                            # park_id_.publish(1)都发2
                            # park_id_.publish(2)
                            print(park_id)
                            break
                        elif jidian == 1 or xinghang == 1:
                            print(park_id)
                            # park_id_.publish(2)
                            break
                        elif cnt == 10 :
                            print('[VISON]-----------识别失败--------------')
                            break
    except rospy.ROSInterruptException:
        cam.release()
    
