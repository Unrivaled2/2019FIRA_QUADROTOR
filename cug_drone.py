# coding=utf-8

import sys
from BasicController import BasicDroneController
import datetime
import cv2
import numpy as np
import time
from edge_detection_canny_multi_rects import *


# define the current and next running model of drone
class DroneStage:
    # drone take off
    TAKEOFF = 0
    # drone perpare to land
    LAND = 1
    # test code
    TEST = 2
    # drone forward along the road
    FORWARD = 3
    # drone run to the top of corner and turn right 90 degrees
    FORWARDEND = 4
    # through down frame
    DOWNFRAME = 5
    # through up frame
    UPFRRAME = 6
    def __init__(self):
        return


class FiraDrone:
    def __init__(self):
        self.stage = DroneStage.TAKEOFF
        self.controller = BasicDroneController()
        self.port = self.controller.port
        # read capture
        self.capture = cv2.VideoCapture(0)
        self.capture.set(3, 640)
        self.capture.set(4, 480)
        for i in range(5):
            self.capture.read()
        self.picCnt = 0
        # height and angle information
        self.alt = self.controller.alt
        self.yaw = 0
        # control
        self.rolErrorList = [0]
        self.yawErrorList = [0]

    def TurnCorner(self):
        initAngle = self.yaw
        desireAngle = initAngle
        self.controller.receive()
        cur_yaw = self.controller.yaw
        if desireAngle < -180:
            desireAngle = desireAngle + 360
        elif desireAngle > 180:
            desireAngle = desireAngle - 360
        print
        'init angle:', initAngle, 'desireAngle', desireAngle
        # start to turn
        cntPrint = 0
        listTime = []
        yawSpeed = 500
        addSpeed = 0
        turnAngle = desireAngle - cur_yaw
        turnDirection = turnAngle / abs(turnAngle)
        while True:
            time1 = int(round(time.time() * 1000))
            self.controller.receive()
            self.yaw = self.controller.yaw
            errorAngle = abs(self.yaw - desireAngle)
            if errorAngle < -180:
                errorAngle = errorAngle + 360
            elif errorAngle > 180:
                errorAngle = 360 - errorAngle
            # turn right,and control speed
            if errorAngle > 60:
                addSpeed = 200
            elif errorAngle > 20:
                addSpeed = 150
            elif errorAngle > 8:
                addSpeed = 100
            elif errorAngle > 3:
                addSpeed = 40
            elif errorAngle < 3:
                addSpeed = 0
            yawSpeed = 500 + addSpeed * turnDirection
            self.controller.setCommand(500, 500, yawSpeed, 500)
            # the drone turn to the appointed direction
            if errorAngle < 3:
                break
            # print information
            cntPrint += 1
            if cntPrint % 10 == 0:
                print
                'angle cur:', self.yaw, 'angle error:', errorAngle, 'yawSpeed:', yawSpeed
            time2 = int(round(time.time() * 1000))
            listTime.append(time2 - time1)
        # compute command interval
        averTime = int(np.mean(listTime))
        maxTime = np.max(listTime)
        maxTimeIndex = np.argmax(listTime)
        print
        'Hover averTime,maxTime,max_dex:', averTime, maxTime, maxTimeIndex
        return

    # Clear buffer
    def Clear(self):
        self.controller.clear()

    # get picture from capture
    def GetImage(self):
        _, frame = self.capture.read()
        return frame

    # save all picture on disk
    def SaveImage(self, frame):
        filename = 'pic/' + str(self.picCnt) + '.jpg'
        if self.picCnt <= 500:
            cv2.imwrite(filename, frame)
            self.picCnt += 1
        return

    # set rol speed depending on the road relative position in the frame

    def move_forward(self):
        """
        本代码用于向前冲刺
        :return:
        """
        self.TurnCorner()
        start_time = time.time()
        while (time.time() - start_time) < 4:
            self.controller.setCommand(roll=500, pitch=660, yaw_velocity=500, z_velocity=500)
        self.controller.setCommand(roll=500, pitch=500, yaw_velocity=500, z_velocity=500)
        return

    def judge_stick_pos(self):
        """
        用于判断棍棒的位置
        :return:
        """
        self.TurnCorner()
        frame = self.GetImage()
        res = process_pictue2(frame)
        processed_image = np.concatenate((frame, res[1]), axis=1)
        self.SaveImage(processed_image)
        centerX = res[2]
        centerY = res[3]
        Point1 = res[5]
        Point2 = res[6]
        edge_point = Point1
        Point1_dis = pow(Point1[0] - centerX,2) + pow(Point1[1] - centerY,2)
        Point2_dis = pow(Point2[0] - centerX,2) + pow(Point2[1] - centerY,2)
        if(Point2_dis > Point1_dis):
            edge_point = Point2
        if(edge_point[0] > centerX) & (edge_point[1] > centerY):
            return 4
        if (edge_point[0] < centerX) & (edge_point[1] > centerY):
            return 3 #bottom left
        if (edge_point[0] > centerX) & (edge_point[1] < centerY):
            return 2
        if (edge_point[0] < centerX) & (edge_point[1] < centerY):
            return 1 #left top
        return 0

    def move_great_distance(self):
        """
        本代码用于移动到合适的距离
        :return:
        """
        while True:
            self.TurnCorner()
            frame = self.GetImage()
            res = process_pictue(frame)
            processed_image = np.concatenate((frame, res[1]), axis=1)
            self.SaveImage(processed_image)
            percent = res[4]
            if(percent > 0.34) & (percent < 0.36):
                print("******************find great distance*********************************")
                self.controller.setCommand(roll=500, pitch=500, yaw_velocity=500, z_velocity=500)
                break
            if(percent <= 0.34):
                self.controller.setCommand(roll=500, pitch=530, yaw_velocity=500, z_velocity=500)
            else:
                self.controller.setCommand(roll=500, pitch=470, yaw_velocity=500, z_velocity=500)
        return

    def move_y_great(self):
        """
        本代码用于移动到Y轴底部
        :return:
        """
        while True:
            self.TurnCorner()
            frame = self.GetImage()
            res = process_pictue(frame)
            processed_image = np.concatenate((frame, res[1]), axis=1)
            self.SaveImage(processed_image)
            centerX = res[2]
            centerY = res[3]
            if(centerY < 240) & (centerY > 220):
                print("**********************find good Y pos**********************")
                self.controller.setCommand(roll=500, pitch=500, yaw_velocity=500, z_velocity=500)
                break
            if(centerY >= 240):
                self.controller.setCommand(roll=500, pitch=500, yaw_velocity=500, z_velocity=470)
            else:
                self.controller.setCommand(roll=500, pitch=500, yaw_velocity=500, z_velocity=530)
        return

    def move_x_center(self):
        """
        本代码用于移动到X轴中心
        :return:
        """
        while True:
            self.TurnCorner()
            frame = self.GetImage()
            res = process_pictue(frame)
            processed_image = np.concatenate((frame, res[1]), axis=1)
            self.SaveImage(processed_image)
            if(res[0] != 1):
                self.see_object()
                continue
            centerX = res[2]
            if(centerX < 330) & (centerX > 310):
                print("***********************move to good X center**************************")
                self.controller.setCommand(roll=500, pitch=500, yaw_velocity=500, z_velocity=500)
                break
            if(centerX < 320):
                self.controller.setCommand(roll=470, pitch=500, yaw_velocity=500, z_velocity=500)
            else:
                self.controller.setCommand(roll=530, pitch=500, yaw_velocity=500, z_velocity=500)
        return

    def move_stable_pos(self):
        """
        移动到光流可以定住的地方
        :return:
        """
        self.TurnCorner()
        start_time = time.time()
        while (time.time() - start_time) < 3:
            self.controller.setCommand(roll=470, pitch=530, yaw_velocity=500, z_velocity=500)
        self.controller.setCommand(roll=500, pitch=500, yaw_velocity=500, z_velocity=500)
        print("***************************move to good start position************************")


    def see_object(self):
        while True:
            self.TurnCorner()
            frame = self.GetImage()
            res = process_pictue(frame)
            result_code = res[0]
            if(result_code == 1):
                percent = res[4]
                if(percent < 0.3):
                    continue
                print("*********************have see object*******************")
                self.controller.setCommand(roll=500, pitch=500, yaw_velocity=500, z_velocity=500)
                processed_image = np.concatenate((frame, res[1]), axis=1)
                self.SaveImage(processed_image)
                break
            else:
                print("*****************seeing object*****************")
                self.controller.setCommand(roll=500, pitch=530, yaw_velocity=500, z_velocity=520)

        return

    def challenge(self):
        """
        本代码为挑战赛运行入口函数
        :return:
        """
        """
                本代码用于最开始找见框
                :return:
                """
        # first control the height of drone
        # self.keep_height(height)
        print
        '****** begin doing task ******'
        print
        '****** take_off ******'
        # clear receive buffer
        self.Clear()
        self.TakeOff()
        self.Clear()
        self.controller.receive()
        self.yaw = self.controller.yaw
        self.KeepHeight(150)
        #self.move_stable_pos()
        #now_time = time.time()
        #while(time.time() - now_time)<2:
            #self.controller.setCommand(roll=500,pitch=500,yaw_velocity=500,z_velocity=550)
        for i in range(3):
            self.see_object()
            self.move_x_center()
            self.move_great_distance()
            self.move_y_great()
        print("输入3")
        a = input()
        if(a == '3'):
            print("**************************begin go forward***************************")
            self.move_forward()
        pos = self.judge_stick_pos()
        if(pos == 3):
            print("**************************begin go forward***************************")
            self.move_forward()
        self.controller.land()
        return

    def SetRolSpeed(self, xError):
        roll_speed = 500
        if (xError is None) or (abs(xError) < 15):
            return 500
        if abs(xError) > 100:
            roll_speed = 570 if (xError > 0) else 430
        elif abs(xError) > 50:
            roll_speed = 540 if (xError > 0) else 460
        elif abs(xError) > 15:
            roll_speed = 520 if (xError > 0) else 480
        return roll_speed

    def SetPitSpeed(self, yError):
        yError = -yError
        pitSpeed = 500
        if (yError is None) or (abs(yError) < 15):
            return 500
        if abs(yError) > 100:
            pitSpeed = 630 if (yError > 0) else 370
        elif abs(yError) > 50:
            pitSpeed = 580 if (yError > 0) else 420
        elif abs(yError) > 15:
            pitSpeed = 540 if (yError > 0) else 460
        return pitSpeed
    
    def SetYawSpeed(self, angle):
        yawSpeed = 500
        kP = 5
        kI = 0.1
        if (angle is None) or (abs(angle) < 2):
            return 500
        if len(self.yawErrorList) >= 5:
            del self.yawErrorList[0]
            self.yawErrorList.append(angle)
        else:
            self.yawErrorList.append(angle)
        errorI = sum(self.yawErrorList)
        yawSpeed = 500 + angle * kP + kI * errorI
        return int(yawSpeed)

    def SetThrSpeed(self, height):
        thrSpeed = 500
        errorHeight = height - self.alt * 100
        # set thr speed depending on the error height,using proportion control
        if abs(errorHeight) > 10:
            thrSpeed = 600 if errorHeight > 0 else 400
        if abs(errorHeight) < 10 or self.alt > 1.5:
            thrSpeed = 500
        return thrSpeed

    # keep the drone in appointed height(xx cm)
    def KeepHeight(self, height):
        # Clear receive buffer
        self.Clear()
        # the drone got too high and out of control,let it still
        print
        '******** control height *********'
        print
        'expected height:',height
        cnt = 0
        thrSpeed = 500
        while True:
            cnt += 1
            self.controller.receive()
            self.alt = self.controller.alt
            errorHeight = height - self.alt * 100
            # set thr speed depending on the error height,using proportion control
            if abs(errorHeight) > 30:
                thrSpeed = 600  if errorHeight > 0 else 400
            elif abs(errorHeight) > 8:
                thrSpeed = 540  if errorHeight > 0 else 460
            else:
                thrSpeed = 500
            if self.alt >= 1.5:
                thrSpeed = 500
            self.controller.setCommand(500, 500, 500, thrSpeed)
            if abs(errorHeight) < 8 :
                break
            # print information
            if cnt % 5 == 0:
                print
                'current height:', self.alt * 100, 'error height:', errorHeight, 'thr speed:', thrSpeed
        return

    def AdjustHeight(self):
        print
        '******** adjust height *********'
        cnt = 0
        thrSpeed = 500
        while True:
            cnt +=1 
            self.controller.receive()
            self.alt = self.controller.alt

        return

    def keep_height(self, height):
        # the drone got too high and out of control,let it still
        print
        '******** control height *********'
        print
        'expected height:', height
        print_cnt = 0
        cnt = 0
        thr_speed = 500
        while True:
            self.controller.receive()
            self.alt = self.controller.alt
            height_error = height - self.alt * 100
            if self.alt >= 1.5:
                thr_speed = 500
            # set thr speed depending on the error height,using proportion control
            elif height_error > 8:
                thr_speed = 600
            elif height_error < -8:
                thr_speed = 400
            elif abs(height_error) < 5:
                thr_speed = 500
            self.controller.setCommand(roll=500, pitch=500, yaw_velocity=500, z_velocity=thr_speed)
            if abs(height_error) < 5 or self.alt > 1.5:
                break
            # print information
            print_cnt += 1
            if print_cnt % 5 == 0:
                print
                'current height:', self.alt * 100, 'error height:', height_error, 'thr speed:', thr_speed
            cnt += 1
            time.sleep(0.05)
        return


        
    # take off and Hover 10 seconds
    def TakeOff(self):
        print('take off.')
        fira.controller.takeoff()
        start_time = time.time()
        while (time.time() - start_time) < 2:
            self.controller.receive()
            self.controller.setCommand(roll=500, pitch=500, yaw_velocity=500, z_velocity=500)
        return



if __name__ == '__main__':
    fira = FiraDrone()
    # main process to run the Game
    fira.challenge()
    # save all picture on disk
    print
    'end.'
