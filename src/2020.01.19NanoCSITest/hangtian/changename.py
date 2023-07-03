# -*- coding:utf-8 _*-

import cv2

for i in range(100):
    img = cv2.imread("/home/cquer/2023_qingzhou/src/2020.01.19NanoCSITest/hangtian/jidian_ %4d.jpg"%i)
    cv2.imwrite("/home/cquer/2023_qingzhou/src/2020.01.19NanoCSITest/hangtian/hangtian_ %4d.jpg"%i,img)
