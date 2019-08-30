#coding=utf8
import time
import string
import binascii
import sys
from serial_exchange import Ser


class BasicDroneController(object):
    def __init__(self):
        self.Publish = Ser()
        self.port = self.Publish.port
        self.Publish.cmd_convert(0, 0) #初始化前4个通道值为500
        self.alt = self.Publish.altitude
        self.yaw = self.Publish.ano_yaw

    def takeoff(self):
        self.Publish.cmd_convert(5,500)
        self.Publish.cmd_convert(7, "takeoff")
        print('takeoff',self.Publish.append_cmd())
        self.Publish.send_cmd(binascii.a2b_hex(self.Publish.append_cmd()))

    def land(self):
        self.Publish.cmd_convert(7, "land")
        print('land',self.Publish.append_cmd())
        self.Publish.send_cmd(binascii.a2b_hex(self.Publish.append_cmd()))
        self.Publish.cmd_convert(0, 0) #发送降落命令后将通道值回到500  第五通道也回到0
        self.Publish.cmd_convert(5, 0)
        self.Publish.send_channel(binascii.a2b_hex(self.Publish.append_channel()))

    def ch_pwm(self,n):
        if n == 1:
            self.Publish.cmd_convert(7, "pwm1")
        if n == 2:
            self.Publish.cmd_convert(7, "pwm2")
        if n == 3:
            self.Publish.cmd_convert(7, "pwm3")
        print('change channel ', n,self.Publish.append_cmd())
        self.Publish.send_cmd(binascii.a2b_hex(self.Publish.append_cmd()))

    def setCommand(self,roll=500,pitch=500,yaw_velocity=500,z_velocity=500):   #rol 1  pit 2 thr 3 yaw 4
        # remove  dead zone
        if (roll >= 500):
            roll = roll + 50
        else:
            roll = roll - 50
        if (pitch >= 500):
            pitch = pitch + 50
        else:
            pitch = pitch - 50
        if (z_velocity >= 500):
            z_velocity = z_velocity + 50
        else:
            z_velocity = z_velocity - 50
        if (yaw_velocity >= 500):
            yaw_velocity = yaw_velocity + 65
        else:
            yaw_velocity = yaw_velocity - 65
        self.Publish.cmd_convert(1,roll)
        self.Publish.cmd_convert(2, pitch)
        self.Publish.cmd_convert(3, z_velocity)
        self.Publish.cmd_convert(4, yaw_velocity)
        self.send_channel()

    def clear(self):
        self.Publish.clear()
        time.sleep(0.01)


    def receive(self):
        self.Publish.receive()
        self.alt = self.Publish.altitude
        self.yaw = self.Publish.ano_yaw

    def hover(self):
        self.Publish.cmd_convert(0, 0)

    def send_channel(self):
        #time.sleep(0.01)
        # print('channel', self.Publish.pit, self.Publish.rol,
        #       self.Publish.thr, self.Publish.yaw,
        #       self.Publish.aux1, self.Publish.aux2, self.Publish.aux3,
        #       self.Publish.aux4)
        self.Publish.send_channel(binascii.a2b_hex(self.Publish.append_channel()))

"""
if __name__ == '__main__':
    controller = BasicDroneController()
    cnt = 0
    while(cnt < 200):
        cnt += 1
        controller.receive()
        print cnt , controller.yaw
"""
    
    

