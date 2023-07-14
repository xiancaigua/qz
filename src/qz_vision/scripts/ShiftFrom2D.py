import numpy as np
import cv2
import matplotlib.pyplot as plt
from openvideo import *
import math

camera_parameter = {
    # f, f
    "f": [607.3162, 606.5926],
    # center point
    "c": [654.1366, 509.9644]
}

def pixel_to_world(camera_intrinsics, img_points,sita):

    K_inv = camera_intrinsics.I
    world_points_before=[]
    world_points_before=np.dot(K_inv,img_points)
    trans = np.zeros((3,3), dtype=np.float64)
    trans[0,0]=math.cos(sita)
    trans[0,1]=-(math.sin(sita))
    trans[1,0]=math.sin(sita)
    trans[1,1]=math.cos(sita)
    world_points_after=np.dot(trans,world_points_before)
    return world_points_after

def getPoint(img,theta=1.57):
    img1 = cv2.imread('images/P.jpg', cv2.IMREAD_GRAYSCALE)  # 索引图像
    # img2 = cv2.imread('wenzi/jidian/jd (43).jpg', cv2.IMREAD_GRAYSCALE)  # 训练图像
    # img4 = cv2.imread('wenzi/jidian/jd (43).jpg', cv2.IMREAD_GRAYSCALE)  # 训练图像
    img2 = img
    img4 = img.copy()
    img2 = img2[500:800, 0:600]  # y:y+h x:x+w
    # 初始化SIFT描述符
    sift = cv2.SIFT_create()
    # 基于SIFT找到关键点和描述符
    kp1, des1 = sift.detectAndCompute(img1, None)
    kp2, des2 = sift.detectAndCompute(img2, None)
    # 默认参数初始化BF匹配器
    bf = cv2.BFMatcher()
    matches = bf.knnMatch(des1, des2, k=2)
    # 应用比例测试
    good = []
    i = -1
    u = 0
    v = 0
    for m, n in matches:
        i = i + 1
        if m.distance < 0.8 * n.distance:
            good.append([m])
            u = kp2[i].pt[0] + u
            v = kp2[i].pt[1] + v
            print(kp2[i].pt)
    u = u / len(good)
    v = v / len(good)
    print(u, v)
    cv2.circle(img2, center=(int(u) + 100, int(v) - 10), radius=5, color=[0, 255, 0], thickness=2)
    if len(good) > 10:
        print('ok')
    else:
        print('no')
    img3 = cv2.drawMatchesKnn(img1, kp1, img2, kp2, good, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    cv2.circle(img4, center=(int(u) + 100, int(v) - 10 + 500), radius=5, color=[0, 255, 0], thickness=2)

    f = camera_parameter["f"]
    c = camera_parameter["c"]
    camera_intrinsic = np.mat(np.zeros((3, 3), dtype=np.float64))
    camera_intrinsic[0, 0] = f[0]
    #camera_intrinsic[0, 1] = -1.3081 #对s比例因子进行修正
    camera_intrinsic[1, 1] = f[1]
    camera_intrinsic[0, 2] = c[0]
    camera_intrinsic[1, 2] = c[1]
    camera_intrinsic[2, 2] = np.float64(1)
    img_points = np.array(([int(u) + 100, int(v) - 15+500, 1]), dtype=np.double)
    img_points=np.asmatrix(img_points).T
    result = pixel_to_world(camera_intrinsic, img_points,theta)

    return result


if __name__ == "__main__":
    cam=cv2.VideoCapture(gstreamer_pipeline(flip_method=0),cv2.CAP_GSTREAMER)
    while cam.isOpened():
        ret,img = cam.read()
        if not ret:
            print("Cam is not Opened")
            break
        result = getPoint(img)
        print(result)