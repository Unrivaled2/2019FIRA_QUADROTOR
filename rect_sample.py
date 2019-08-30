# -*- coding: utf-8 -*-

import cv2
import numpy as np


def hsv_sample(event,x,y,flags,param):
	global max_hsv,min_hsv,min_list,max_list,img_init,img_hsv,img_mask,img_tem,img_show, img_rgb
	#左键单击记录矩形角点
	if event == cv2.EVENT_LBUTTONDOWN:
		rect[0] = [x,y]
		
	#左键拖拽
	if flags == cv2.EVENT_FLAG_LBUTTON:
		rect[1] = [x,y]
		#绘制矩形
		img_show = np.copy(img_tem)
		cv2.rectangle(img_show,(rect[0][0],rect[0][1]),(rect[1][0],rect[1][1]), (0, 255, 0), 1)
	
	#左键释放
	if event == cv2.EVENT_LBUTTONUP:
		#绘制矩形
		rect[1] = [x,y]
		cv2.rectangle(img_tem,tuple(rect[0]),tuple(rect[1]), (0, 255, 0), 1)
		img_show = np.copy(img_tem)
		#确定矩形范围
		x_min = min(rect[0][0], rect[1][0])
		y_min = min(rect[0][1], rect[1][1])
		x_max = max(rect[0][0], rect[1][0])
		y_max = max(rect[0][1], rect[1][1])
		#计算HSV的范围
		img_rect = img_hsv[y_min:y_max, x_min:x_max,:]
		min_tem = np.min(np.min(img_rect,axis=0),axis=0)
		max_tem = np.max(np.max(img_rect,axis=0),axis=0)
		min_list.append(min_tem)
		max_list.append(max_tem)
		# compute RGB range
		img_rect_rgb = img_rgb[y_min:y_max, x_min:x_max,:]
		min_rgb = np.min(np.min(img_rect_rgb,axis=0),axis=0)
		max_rgb = np.max(np.max(img_rect_rgb,axis=0),axis=0)
		#识别轮廓,contours为list类型,保存所有的轮廓.其中每条轮廓为ndarray类型
		#contours 结构为 
		min_hsv = np.min(min_list,axis=0)
		max_hsv = np.max(max_list,axis=0)
		img_mask = cv2.inRange(img_hsv,min_hsv,max_hsv)
		#用膨胀和腐蚀,对二值图像进行预处理
		kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5, 5))
		img_mask = cv2.dilate(img_mask,kernel) #膨胀
		#识别物体轮廓
		contours, hierarchy = cv2.findContours(img_mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		cv2.drawContours(img_show,contours,-1,(255,0,0),1) 
		rect_cnt = 0
		for cnt in contours:
			if len(cnt) > 200:
				x,y,w,h = cv2.boundingRect(cnt)
				cv2.rectangle(img_show,(x,y),(x+w,y+h),(0,255,),3)
				cv2.circle(img_show,(int(x+w/2),int(y+h/2)),2,(0,0,255),2)
				rect_cnt += 1
		#输出HSV范围
		print ('HSV:', min_hsv,max_hsv,rect_cnt, 'RGB:', min_rgb, max_rgb)
		
		
	#中间点击,绘制轮廓
	if event == cv2.EVENT_MBUTTONDOWN:
		img_show = np.copy(img_init)
		img_tem = np.copy(img_init)
		img_mask = np.copy(img_hsv)
		del min_list[:]
		del max_list[:]
		
if __name__=='__main__':	
	#初始化各图像
	img_init = cv2.imread("C:\\Users\\18056\\PycharmProjects\\untitled\\air_drone_vertical\\pic1\\300.jpg",1)
	img_hsv = cv2.cvtColor(img_init,cv2.COLOR_BGR2HSV)
	img_show = np.copy(img_init)
	img_tem = np.copy(img_init)
	img_rgb = np.copy(img_init)
	img_mask = np.copy(img_hsv)
	cv2.namedWindow('img')
	cv2.namedWindow('mask')
	#设置鼠标响应函数
	cv2.setMouseCallback('img',hsv_sample)
	#定义选取矩形框和hsv范围
	rect = [[0,0],[0,0]]
	max_hsv = np.array([0,0,0])
	min_hsv = np.array([255,255,255])
	max_list = []
	min_list = []
	#循环
	while(1):
		cv2.imshow('img',img_show)
		cv2.imshow('mask',img_mask)
		if cv2.waitKey(20) &0xFF == 27:
			break
		
	cv2.destroyAllWindows()


