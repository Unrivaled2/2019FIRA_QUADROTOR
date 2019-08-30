# -*- coding:utf-8 -*-

import cv2
import numpy as np
import time

if __name__=='__main__':
    cap = cv2.VideoCapture(0)
    print
    'fps:',cap.get(cv2.CAP_PROP_FPS)

    cnt = 0
    frame = None
    while cnt < 900:
        ret, frame = cap.read()
        # write line
        #cv2.line(frame, (0, 240), (640, 240), (255, 0, 0), 2)
        #cv2.line(frame, (320, 0), (320, 480), (255, 0, 0), 2)
        #img = cv2.circle(frame,(320,240),2,(0,0,255),3)
        #cv2.imshow('frame',frame)
        if cv2.waitKey(1) and 0xff == ord('q'):
            break
        file_name = 'pic1/' + str(cnt) + '.jpg'
        cv2.imwrite(file_name,frame)
        cnt += 1
    cap.release()



def detect():
    cnt = 0
    while cnt < 6:
        ret = cap.grab()
        cnt += 1
        print
        ret
    # print 'time:',time.clock()-start
    ret, frame = cap.read()
