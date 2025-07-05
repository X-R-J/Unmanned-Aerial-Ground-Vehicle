import Hobot.GPIO as GPIO
import time

DB = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]    # 16位并行数据线
RD = 17          # 芯片读信号，低电平有效
CONVST = 18      # 转换开始信号，低电平有效
RST = 19         # 芯片复位，低电平有效
A0, A1, A2 = 20, 21, 22   # 74HC138控制信号
E1 = 23    # 74HC138使能信号，低电平有效

class AD7606_Controller:
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        for pin in DB:
            GPIO.setup(pin,GPIO.IN)
        GPIO.setup(RD,GPIO.OUT,initial = GPIO.HIGH)
        GPIO.setup(CONVST,GPIO.OUT,initial = GPIO.HIGH)
        GPIO.setup(A0,GPIO.OUT,initial = GPIO.LOW)
        GPIO.setup(A1,GPIO.OUT,initial = GPIO.LOW)
        GPIO.setup(A2,GPIO.OUT,initial = GPIO.LOW)
        GPIO.setup(E1,GPIO.OUT,initial = GPIO.HIGH)
        GPIO.setup(RST,GPIO.OUT,initial = GPIO.LOW)
        time.sleep(0.001)
        GPIO.output(RST,GPIO.HIGH)
        time.sleep(0.001)

    def select_chip(self,chip_num):
        binary = format(chip_num - 1, '03b')
        GPIO.output(A2,int(binary[0])^1)
        GPIO.output(A1,int(binary[1])^1)
        GPIO.output(A0,int(binary[2])^1)

    def start_conversion(self):
        GPIO.output(CONVST,GPIO.LOW)
        GPIO.output(CONVST,GPIO.HIGH)

    def read_vol(self,chip_num):
        self.select_chip(chip_num)
        GPIO.output(E1,GPIO.LOW)
        voltage = [0]*8
        for i in range(8):
            GPIO.output(RD,GPIO.LOW)
            ad_value = 0
            for j in range(16):
                ad_value |= (GPIO.input(DB[j]))<<j
            voltage[i] = int(ad_value/32768.0*10.0*1000.0)
            GPIO.output(RD,GPIO.HIGH)
        GPIO.output(E1,GPIO.HIGH)
        return voltage
        