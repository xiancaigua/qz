#!/home/cquer/archiconda3/envs/qz/bin python
#coding=utf-8
"""
1. 配置环境
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
在此目录下pip install -r reuirements.txt 
2. 调用时, 将此文件夹放到调用文件的文件夹下,
from yolo_detect.detect import detect
detect(onnx_path='best.onnx', img_path='text.jpg', show=False)
- onnx_path: 模型的路径, 可以不传
- img_path: 注意不要直接传图片, 要把图片先保存再传入路径(不然要改动的有点多, 我怕写错)
(相对于此脚本的路径, 建议直接保存到当前目录, 命名为test.jpg)
- show: 是否画图, 默认不显示
返回拼音

"""
from PIL import Image,ImageDraw
import time
import sys
sys.path.append('/home/cquer/2023_qingzhou/src/qz_vision/scripts/yolo_detect')
from utils.operation import YOLO
import cv2
import onnxruntime

class Detect():
    def __init__(self, object,
            onnx_path='/home/cquer/2023_qingzhou/src/qz_vision/scripts/yolo_detect/best2.onnx'):
        assert object in ['traffic', 'text'], 'object should be traffic or text'
        self.object = object
        if (self.object == 'traffic'):
            onnx_path='/home/cquer/2023_qingzhou/src/qz_vision/scripts/yolo_detect/traffic_mix.onnx'
        self.onnx_session = onnxruntime.InferenceSession(onnx_path)
        
    def detect(self, img, show=False):
        '''
        检测目标，返回目标所在坐标如：
        {'crop': [57, 390, 207, 882], 'classes': 'person'},...]
        :param onnx_path:onnx模型路径
        :param img:检测用的图片
        :param show:是否展示
        :return:
        '''
        yolo = YOLO(self.onnx_session, self.object)
        # t1 = time.time()
        det_obj = yolo.decect(img)
        # t2 = time.time()
        # print("跑模型用时：", t2-t1)

        # 结果
        # print (det_obj)
        if len(det_obj) == 0:
            return ""

        # 画框框
        if show:
            draw = ImageDraw.Draw(img)

            for i in range(len(det_obj)):
                draw.rectangle(det_obj[i]['crop'],width=3)
            img.show()  # 展示
        return det_obj[0]

if __name__ == "__main__":
    img = cv2.imread('text.jpg')
    # print(type(Image.open('text.jpg')))
