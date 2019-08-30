#!/usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import numpy as np
import time
import math
from air_drone_vertical.edge_detection_canny_multi_rects import *

if __name__ == '__main__':
    cnt = 0
    while cnt < 600:
        file_name = 'C:\\Users\\18056\\PycharmProjects\\untitled\\air_drone_vertical\\pic1\\' + str(cnt) + '.jpg'
        print(file_name)
        img  = cv2.imread(file_name)
        if (img is None):
            cnt += 1
            continue
        #cross_detect time compute
        img1 = np.copy(img)
        mark = process_pictue2(img)
        print("结果编码： " + str(mark[0]))

        if (mark[0] == 12):
            print("未找到中心点")
            cnt += 1
            continue
        else:
            print("成功找到可信中心点")
            image = mark[1]
            percent = mark[4]
            print("percent: " + str(mark[4]))
            print("centerX: " + str(mark[2]))
            print("centerY: " + str(mark[3]))
        #compute time
        cv2.imshow('origin',img1)
        cv2.imshow('process',image)
        if cv2.waitKey(0) == 27:
            break
        cv2.destroyWindow('origin')
        cv2.destroyWindow('process')
        cnt += 1
    cv2.destroyAllWindows()
