#!/usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import numpy as np  
import time
import math

class Rectify():
    def __init__(self):
        self.mapx = None
        self.mapy = None
        self.imgRectify = None
        cameraMatrix = np.mat([[362.7836, 0, 317.814235],
                    [0, 365.391025, 226.170877],
                    [0, 0, 1]])
        distCoeffs = np.mat([[-0.300412], [0.061103], [0.006351], [0.001000], [0]])
        R = np.eye(3, k = 0)
        ncm, _ = cv2.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, (640, 480), 1)
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(cameraMatrix, distCoeffs, R, ncm, (640, 480), cv2.CV_32FC1) 

    def RectifyImage(self, image):
        self.imgRectify = cv2.remap(image, self.mapx, self.mapy, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT_101)
        #self.imgRectify = self.imgRectify[60:420, 40:600]
        self.imgRectify = self.imgRectify[60:420, 120:520]
        return self.imgRectify

if __name__=='__main__':
    rectify = Rectify() 
    cnt = 200
    while cnt < 300:
        cnt += 1
        file_name = 'C:\\Users\\18056\\PycharmProjects\\untitled\\air_drone\\withSkew\\' + str(cnt) + '.jpg'
        #file_name = 'test.jpg'
        img = cv2.imread(file_name)
        if img is None:
            print
            'no image'
            continue
        imgRectify = rectify.RectifyImage(img)
        cv2.imshow('imgRectify ', imgRectify)
        cv2.imshow('img ', img)
        if cv2.waitKey(0) == 27:
            break
   
