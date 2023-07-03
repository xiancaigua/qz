#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

import cv2
import torch
from my_model import MyModel
import torchvision.transforms as transforms

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

# 自定义参数
class Args:
    def __init__(self) -> None:
        self.batch_size = 64
        self.test_batch = 1
        self.lr = 1e-3
        self.epochs = 30
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# 图片缩放函数
def crop(img):
    row, col, _ = img.shape
    cropped = img[int(row*0.45):int(row-row*0.32),
                    int(col*0.2):int(col-col*0.2)]
    return cropped

##########
args = Args()
model = MyModel()
state_dict = torch.load('/home/cquer/2023_qingzhou/src/qz_vision/models/model2.pth')
model.load_state_dict(state_dict)
#########

def torch_recg(img):
    global args,model
    img = crop(img)
    label_dict = {
        0 : 'hangtian',
        1 : 'jidian',
        2 : 'sanyuan',
        3 : 'xinghang',
    }

    test_transform = transforms.Compose([
        transforms.ToPILImage(),
        transforms.Resize((128, 128)),
        transforms.ToTensor(),
    ])
    img = test_transform(img).unsqueeze(0)
    model.to(args.device)
    model.eval()
    print('model prepared')
    with torch.no_grad():
        img = img.to(args.device)

        output = model(img)
        pre_label = output.argmax(1).cpu()
        
        return label_dict[pre_label.item()]




if __name__ == "__main__":
    cam = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    print('-------------------initialized')
    cnt=0
    while cam.isOpened():
        ret,frame = cam.read()
        if not ret:
                break
        print('----------------------------detecting')
        pre = torch_recg(frame)
        cnt+=1
        if cnt==100:
            print(pre)
            cnt=0
            # print('saved!')
            # print(frame.shape)
            # print(crop(frame).shape)
            # cv2.imwrite('/home/cquer/2023_qingzhou/src/qz_vision/pics/2021629.jpg',crop(frame))
            # break
          
          