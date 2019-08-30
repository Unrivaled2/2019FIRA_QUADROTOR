# coding=utf-8

import sys
from BasicController import BasicDroneController
import time
import numpy as np
import cv2

def reciecve(ser):
    cnt = 0
    tStart = time.time()
    ser.clear()
    tEnd = time.time()
    print 'clear data time:', tEnd - tStart
    #time.sleep(3)
    print 'start recieve'
    tStart = time.time()
    while(cnt < 100):
        cnt += 1
        ser.setCommand(500, 500, 500, 500)
        #ser.receive()
        #if cnt%10 == 0:
        #    print cnt, ser.alt, ser.yaw
    tEnd = time.time()
    averTime = (tEnd - tStart) / cnt
    print 'total time:', (tEnd - tStart)
    print 'average time:', averTime

def testCamera():
    capture = cv2.VideoCapture(0)
    capture.set(3, 640)
    capture.set(4, 480)
    cnt = 0
    capture.read()
    time.sleep(10)
    tStart = time.time()
    timeList = []
    while cnt < 100:
        cnt += 1
        tCurrent = time.time()
        _, frame = capture.read()
        timeList.append( int((time.time() - tCurrent)*1000) )
        time.sleep(0.03)
    tEnd = time.time()
    averTime = (tEnd - tStart) / cnt
    averRead = np.mean(timeList)
    minRead = np.min(timeList)
    print 'averTime:', averTime
    print 'averRead:', averRead
    print 'minRead:', minRead
    for i in range(10):
        print timeList[i*10:(i*10 + 10)]

def pwm(ser):
    print 'change pwm.'
    a = input('input channel(1,2,3): ')
    n = int(a)
    ser.ch_pwm(n)

if __name__ == '__main__':
    #test_ser = BasicDroneController()
    #pwm(test_ser)
    #reciecve(test_ser)
    testCamera()
