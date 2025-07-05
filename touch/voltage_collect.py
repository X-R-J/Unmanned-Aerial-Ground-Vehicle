import Hobot.GPIO as GPIO
import time
import struct
import socket
import atexit
import threading
import AD7606

DB = [8,10,5,7,11,13,15,19,21,23,27,29,31,33,35,37]
RD = 16
CONVST = 12
RST = 22
A0, A1, A2 = 24, 28, 38
E1 = 36

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
        GPIO.setup(RST,GPIO.OUT,initial = GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(RST,GPIO.LOW)
        time.sleep(0.001)

    def select_chip(self,chip_num):
        binary = format(chip_num - 1, '03b')
        GPIO.output(A2,GPIO.LOW if int(binary[0])^1 else GPIO.HIGH)
        GPIO.output(A1,GPIO.LOW if int(binary[1])^1 else GPIO.HIGH)
        GPIO.output(A0,GPIO.LOW if int(binary[2])^1 else GPIO.HIGH)

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

def cleanup():
    """清理GPIO和socket资源"""
    print("\nCleaning up resources...")
    try:
        GPIO.cleanup()
        print("GPIO cleaned up")
    except Exception as e:
        print(f"Error cleaning GPIO: {e}")
    
    try:
        if 'server' in globals():
            server.close()
            print("Socket closed")
    except Exception as e:
        print(f"Error closing socket: {e}")

# 注册退出处理函数
atexit.register(cleanup)  

def main():
    global server, conn
    
    AD7606 = AD7606_Controller()
    IP = "0.0.0.0"
    PORT = 8888

    try:
        # 设置socket并允许地址重用
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((IP, PORT))
        server.listen(1)  # 限制最大连接数为1
        server.settimeout(5.0)  # 设置accept超时
        print(f"Server listening on {IP}:{PORT}")
        
        while True:
            try:
                # 等待客户端连接
                conn, addr = server.accept()
                conn.settimeout(5.0)  # 设置recv/send超时
                print(f"Client connected: {addr}")
                
                while True:
                    try:
                        # 采集数据
                        AD7606.start_conversion()
                        v1 = AD7606.read_vol(1)
                        voltage_data = v1
                        
                        # 发送数据
                        data_str = ",".join(map(str, voltage_data))
                        print("Sending:", voltage_data)
                        try:
                            conn.sendall(data_str.encode())
                        except (BrokenPipeError, ConnectionResetError) as e:
                            print(f"Connection error: {e}")
                            break  # 跳出内部循环，重新等待连接
                            
                        
                    except socket.timeout:
                        print("Send timeout, reconnecting...")
                        break
                    except Exception as e:
                        print(f"Error in data loop: {e}")
                        break
                        
            except socket.timeout:
                print("Waiting for client connection...")
                continue
            except Exception as e:
                print(f"Connection error: {e}")
                time.sleep(1)  # 等待后重试
                
    except KeyboardInterrupt:
        print("\nServer stopped by user")
    except Exception as e:
        print(f"Fatal error: {e}")
    finally:
        cleanup()

if __name__ == "__main__":
    main()
