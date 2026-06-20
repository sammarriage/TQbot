# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author    : Yunlong Lian, PhD students
# File      : imu.py
# Date      : 18-Dec-2023
# Version:  : 
# ----------------------------------------------------------------
import serial
import struct
import platform
import serial.tools.list_ports
import math
import time

class Imu:
    def __init__(self, port, baudrate, timeout=0.5):
        self.python_version = platform.python_version()[0]
        self.key = 0
        self.flag = 0
        self.buff = {}
        self.angularVelocity = [0, 0, 0]
        self.acceleration = [0, 0, 0]
        self.magnetometer = [0, 0, 0]
        self.angle_degree = [0, 0, 0]
        self.pub_flag = [True, True]
        self.acc_k = 1

        try:
            self.hf_imu = self.init_serial_port(port, baudrate, timeout)
            if self.hf_imu.isOpen():
                print("\033[32m Serial port is opened successfully...\033[0m")
            else:
                self.hf_imu.open()
                print("\033[32m Serial port is opened successfully...\033[0m")
        except Exception as e:
            print(e)
            print("\033[31m Serial port opening failed...\033[0m")
            # exit(0)

    def find_imu_port(self):
        print('The default port of imu is /dev/ttyIMU0 or /dev/ttyIMU1')
        posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
        print('The current serial port devices are {}: {}'.format(len(posts), posts))

    def init_serial_port(self, port, baudrate, timeout=0.5):
        self.find_imu_port()
        return serial.Serial(port=port, baudrate=baudrate, timeout=timeout)

    def checkSum(self, list_data, check_data):
        data = bytearray(list_data)
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for i in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return hex(((crc & 0xff) << 8) + (crc >> 8)) == hex(check_data[0] << 8 | check_data[1])

    def hex_to_ieee(self, raw_data):
        ieee_data = []
        raw_data.reverse()
        for i in range(0, len(raw_data), 4):
            data2str = hex(raw_data[i] | 0xff00)[4:6] + hex(raw_data[i + 1] | 0xff00)[4:6] + hex(raw_data[i + 2] | 0xff00)[4:6] + hex(raw_data[i + 3] | 0xff00)[4:6]
            if self.python_version == '2':
                ieee_data.append(struct.unpack('>f', data2str.decode('hex'))[0])
            if self.python_version == '3':
                ieee_data.append(struct.unpack('>f', bytes.fromhex(data2str))[0])
        ieee_data.reverse()
        return ieee_data

    def read(self):
        try:
            buff_count = self.hf_imu.inWaiting()
        except Exception as e:
            print("exception:" + str(e))
            print("imu is disconnected")
            # exit(0)
        else:
            if buff_count > 0:
                buff_data = self.hf_imu.read(buff_count)
                for i in range(0, buff_count):
                    self._update(buff_data[i])

        acc = [self.acceleration[i] * -9.8 / self.acc_k for i in range(3)]
        converted_ang = [self.angle_degree[0], -self.angle_degree[1], -self.angle_degree[2]]
                # print("return data")
        return acc, \
               self.angularVelocity, \
               converted_ang, \
               self.magnetometer

    def _update(self, raw_data):
        if self.python_version == '2':
            self.buff[self.key] = ord(raw_data)
        if self.python_version == '3':
            self.buff[self.key] = raw_data

        self.key += 1
        if self.buff[0] != 0xaa:
            self.key = 0
            return
        if self.key < 3:
            return
        if self.buff[1] != 0x55:
            self.key = 0
            return
        if self.key < self.buff[2] + 5:
            return
        else:
            data_buff = list(self.buff.values())
            if self.buff[2] == 0x2c and self.pub_flag[0]:
                if self.checkSum(data_buff[2:47], data_buff[47:49]):
                    data = self.hex_to_ieee(data_buff[7:47])
                    self.angularVelocity = data[1:4]
                    self.acceleration = data[4:7]
                    self.magnetometer = data[7:10]
                else:
                    print('CRC check failed 1')
                self.pub_flag[0] = False
            elif self.buff[2] == 0x14 and self.pub_flag[1]:
                if self.checkSum(data_buff[2:23], data_buff[23:25]):
                    data = self.hex_to_ieee(data_buff[7:23])
                    self.angle_degree = data[1:4]
                else:
                    print('CRC check failed 2')
                self.pub_flag[1] = False
            else:
                print("The data processing hasn't provide the parse of " + str(self.buff[2]))
                print("or wrong data")
                self.buff = {}
                self.key = 0

            self.buff = {}
            self.key = 0
            if self.pub_flag[0] == True or self.pub_flag[1] == True:
                return
            self.pub_flag[0] = self.pub_flag[1] = True
            self.acc_k = math.sqrt(self.acceleration[0] ** 2 + self.acceleration[1] ** 2 + self.acceleration[2] ** 2)
            # acceleration:
            # self.acceleration[0] * -9.8 / acc_k, self.acceleration[1] * -9.8 / acc_k, self.acceleration[2] * -9.8 / acc_k
    #         print('''
    # 加速度(m/s²)：
    #     x轴：%.2f
    #     y轴：%.2f
    #     z轴：%.2f
    #
    # 角速度(rad/s)：
    #     x轴：%.2f
    #     y轴：%.2f
    #     z轴：%.2f
    #
    # 欧拉角(°)：
    #     x轴：%.2f
    #     y轴：%.2f
    #     z轴：%.2f
    #
    # 磁场：
    #     x轴：%.2f
    #     y轴：%.2f
    #     z轴：%.2f
    # ''' % (self.acceleration[0] * -9.8 / acc_k, self.acceleration[1] * -9.8 / acc_k, self.acceleration[2] * -9.8 / acc_k,
    #        self.angularVelocity[0], self.angularVelocity[1], self.angularVelocity[2],
    #        self.angle_degree[0], self.angle_degree[1], self.angle_degree[2],
    #        self.magnetometer[0], self.magnetometer[1], self.magnetometer[2]
    #        ))

if __name__ == "__main__":
    imu = Imu("/dev/ttyUSB1", 921600)
    start_time = time.perf_counter()
    t = 0
    while t < 100:
        last_time = time.perf_counter()
        acc, angv, angd, mag = imu.read()
        # print("X-Y-Z")
        # print("acceleration:")
        # print("{:.2f}\n{:.2f}\n{:.2f}".format(acc[0], acc[1], acc[2]))
        # print("angular Velocity: ")
        # print("{:.2f}\n{:.2f}\n{:.2f}".format(angv[0], angv[1], angv[2]))
        print("Euler angles:")
        print("X: {:.2f}\nY: {:.2f}\nZ: {:.2f}".format(angd[0], angd[1], angd[2]))
        # print("magnetometer:")
        # print("{:.2f}\n{:.2f}\n{:.2f}".format(mag[0], mag[1], mag[2]))
        while time.perf_counter() - last_time < 0.005:
            pass
        t += 1
    print("time: ", time.perf_counter()-start_time)
    print("total times: ", t)