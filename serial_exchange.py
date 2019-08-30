# coding:utf-8
import serial
import string
import binascii
import math
import time

class Ser:
    def __init__(self):
        #channel
        self.thr = 500
        self.yaw = 500
        self.pit = 500
        self.rol = 500
        self.aux1 = 0
        self.aux2 = 0
        self.aux3 = 0
        self.aux4 = 0

        #高度与角度信息
        self.altitude = 0
        self.ano_yaw = 0  #后期测试    需要知道初始角度

        #速度数据
        self.x_speed = 0

        #cmd
        self.cmd1 = '01'
        self.cmd2 = '0000'

        self.data_hex = 'AA1005' #帧头  发送设备  目标设备
        self.data_int = 191
        self.fun_word_and_len_channel = '0310'
        self.fun_word_and_len_cmd = 'E003'
        # 打开端口
        #self.port = serial.Serial(port='COM16', baudrate=115200, bytesize=8, parity='N', stopbits=1, writeTimeout=0)
        #self.port = serial.Serial(port="/dev/serial0", baudrate=115200, writeTimeout=0)
        self.port = serial.Serial(port="/dev/ttyS1", baudrate=115200, writeTimeout=0)

    # 发送channel指令
    def send_channel(self, cmd):
        time.sleep(0.01)   #每隔10ms发送一次数据
        try:
            self.port.write(cmd)
        except (OSError,serial.SerialException,serial.SerialTimeoutException):
            print('channel Except error ')

    # 发送控制指令
    def send_cmd(self, cmd):
        time.sleep(0.01) #每隔10ms发送一次数据
        try:
            self.port.write(cmd)
        except (OSError,serial.SerialException,serial.SerialTimeoutException):
            print('channel Except error ')

    #清空缓冲区
    def clear(self):
        self.port.flushInput()

    # 接收
    def receive(self):
        data = ''
        list = []
        while self.port.inWaiting() > 0:
            if self.port.inWaiting >0:
                data_str = binascii.b2a_hex(self.port.read(1))  # 十六进制转换binascii.b2a_hex()
                data += data_str
                list.append(data_str)
        if data != '':
            #print('len',len(list))
            self.Get_Att_Info(list)
            # print('alt:', self.altitude, '    yaw:', self.ano_yaw)

    # 接收转码
    def Get_Att_Info(self, list):
        if len(list) >= 12:  #速度数据的长度至少为12
            for i in range(len(list)):
                if str(list[i]) == "aa" and i + 3 < len(list):  #帧头
                    if str(list[i + 3]) == "01":  # 姿态数据
                        if i + 17 < len(list):
                            self.altitude = self.Alt_Info(str(list[i + 11]) + str(list[i + 12]) +
                                                          str(list[i + 13]) + str(list[i + 14]))
                            self.ano_yaw = self.Yaw_Info(str(list[i + 9]) + str(list[i + 10]))
                    elif str(list[i + 3]) == "0b":  # 速度数据
                        if i + 11 < len(list):
                            self.x_speed = self.Speed_Info(str(list[i + 7]) + str(list[i + 8]))

    # 高度信息
    def Alt_Info(self, msg):
        temp = 0
        for i in range(len(msg)):
            if msg[i] >= 'a' and msg[i] <= 'z':
                temp += (ord(msg[i]) - 87) * math.pow(16, 7 - i)
            elif msg[i] >= 'A' and msg[i] <= 'Z':
                temp += (ord(msg[i]) - 55) * math.pow(16, 7 - i)
            elif msg[i] >= '0' and msg[i] <= '9':
                temp += (ord(msg[i]) - 48) * math.pow(16, 7 - i)
        if temp >= 2147483647:
            temp = temp - 4294967295 - 1
        # self.altitude = temp/100.0
        return temp / 100.0

    # 角度信息
    def Yaw_Info(self, msg):
        temp = 0
        for i in range(len(msg)):
            if msg[i] >= 'a' and msg[i] <= 'z':
                temp += (ord(msg[i]) - 87) * math.pow(16, 3 - i)
            elif msg[i] >= 'A' and msg[i] <= 'Z':
                temp += (ord(msg[i]) - 55) * math.pow(16, 3 - i)
            elif msg[i] >= '0' and msg[i] <= '9':
                temp += (ord(msg[i]) - 48) * math.pow(16, 3 - i)
        if temp >= 32678:
            temp = temp - 65535 - 1
        return temp / 100.0
        # self.ano_yaw = temp/100.0

    # 速度信息
    def Speed_Info(self, msg):
        temp = 0
        for i in range(len(msg)):
            if msg[i] >= 'a' and msg[i] <= 'z':
                temp += (ord(msg[i]) - 87) * math.pow(16, 3 - i)
            elif msg[i] >= 'A' and msg[i] <= 'Z':
                temp += (ord(msg[i]) - 55) * math.pow(16, 3 - i)
            elif msg[i] >= '0' and msg[i] <= '9':
                temp += (ord(msg[i]) - 48) * math.pow(16, 3 - i)
        if temp >= 32768:
            temp = temp - 65535 - 1
        return temp / 100.0

    def channel1_4_reset(self):  #重置4个通道数据的值为500
        self.rol = 500
        self.yaw = 500
        self.thr = 500
        self.pit = 500

    def ROL(self,msg):  # 1
        self.rol = int(msg)

    def PIT(self,msg):  # 2
        self.pit = int(msg)

    def THR(self,msg):  # 3
        self.thr = int(msg)

    def YAW(self,msg):  # 4
        self.yaw = int(msg)

    def Aux1(self,msg):#5
        self.aux1 = int(msg)

    def Aux2(self,msg):#6
        self.aux2 = int(msg)

    def CMD2(self,msg):#6   起飞降落丢包指令数据
        if msg == "takeoff":
            self.cmd2 = '00A2'
        elif msg == 'land':
            self.cmd2 = '00A3'
        elif msg == 'pwm1':
            self.cmd2 = '00A4'
        elif msg == 'pwm2':
            self.cmd2 = '00A5'
        elif msg == 'pwm3':
            self.cmd2 = '00A6'


    #十六进制补齐0并转字符串
    def hex_to_str(self,num):
        num = str(hex(num))[2:]
        return (4 - len(num)) * '0' + num
        # if len(num) == 1:
        #     return '000'+num
        # elif len(num) == 2:
        #     return '00'+num
        # elif len(num) == 3:
        #     return '0' + num

    #对十六进制的字符串进行转整处理
    def str_to_int(self,s):
        temp1 = temp2 = 0
        if s[0].isdigit():
            temp1 += (ord(s[0])-48)*16
        elif s[0]>='A' and s[0] <='Z':
            temp1 += (ord(s[0])-65+10)*16
        elif s[0]>='a' and s[0] <='z':
            temp1 += (ord(s[0])-97+10)*16
        if s[1].isdigit():
            temp2 += ord(s[1])-48
        elif s[1] >= 'A' and s[1] <= 'Z':
            temp2 += ord(s[1])-65+10
        elif s[1] >= 'a' and s[1] <= 'z':
            temp2 += ord(s[1]) - 97 + 10
        return temp1+temp2

    #通道值传输的校验位
    def add_hex_channel(self):
        temp = 0
        self.hex = []
        self.hex.append(self.data_hex)
        self.hex.append(self.fun_word_and_len_channel)
        self.hex.append(self.hex_to_str(self.rol))
        self.hex.append(self.hex_to_str(self.pit))
        self.hex.append(self.hex_to_str(self.thr))
        self.hex.append(self.hex_to_str(self.yaw))
        self.hex.append(self.hex_to_str(self.aux1))
        self.hex.append(self.hex_to_str(self.aux2))
        self.hex.append(self.hex_to_str(self.aux3))
        self.hex.append(self.hex_to_str(self.aux4))
        for i in self.hex:
            for j in range(0,len(str(i)),2):
                temp += self.str_to_int(str(i)[j:j+2])
        return str(hex(temp)[-2:])

    #待发送的十六进制通道值数据
    def append_channel(self):
        return self.data_hex + self.fun_word_and_len_channel + self.hex_to_str(self.rol)+ \
               self.hex_to_str(self.pit)+ self.hex_to_str(self.thr)\
               + self.hex_to_str(self.yaw)+ self.hex_to_str(self.aux1)+\
               self.hex_to_str(self.aux2) + self.hex_to_str(self.aux3)+\
               self.hex_to_str(self.aux4)+self.add_hex_channel()

    # 通道值传输的校验位
    def add_hex_cmd(self):
        temp = 0
        self.hex = []
        self.hex.append(self.data_hex)
        self.hex.append(self.fun_word_and_len_cmd)
        self.hex.append(self.cmd1)
        self.hex.append(self.cmd2)
        for i in self.hex:
            for j in range(0,len(str(i)),2):
                temp += self.str_to_int(str(i)[j:j+2])
        return str(hex(temp)[-2:])

    #待发送的十六进制命令数据
    def append_cmd(self):
        return self.data_hex + self.fun_word_and_len_cmd + self.cmd1\
               + self.cmd2 +self.add_hex_cmd()

    def cmd_convert(self, num, msg):
        if num == 0:
            self.channel1_4_reset()
        if num == 1:
            self.ROL(msg)
        if num == 2:
            self.PIT(msg)
        if num == 3:
            self.THR(msg)
        if num == 4:
            self.YAW(msg)
        if num == 5:
            self.Aux1(msg)
        if num == 6:
            self.Aux2(msg)
        if num == 7:
            self.CMD2(msg)






