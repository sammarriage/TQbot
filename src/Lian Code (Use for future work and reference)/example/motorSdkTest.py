# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author    : Yunlong Lian, PhD students
# File      : motorSdkTest.py
# Date      : 17-June-2024
# Version:  : 3.0.0
# ----------------------------------------------------------------
import time
import os
import sys
actuator_adk_path = '../../../unitree_actuator_sdk/lib'
sys.path.append(os.path.abspath(actuator_adk_path))
from unitree_actuator_sdk import *

serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()

scmd = MotorCmd()
sdata = MotorData()
sdata.motorType = MotorType.A1
scmd.motorType = MotorType.A1
scmd.mode = queryMotorMode(MotorType.A1, MotorMode.BRAKE)
scmd.id   = 0
for i in range(4):
    data.motorType = MotorType.A1
    cmd.motorType = MotorType.A1
    cmd.mode = queryMotorMode(MotorType.A1,MotorMode.FOC)
    cmd.id   = 0
    cmd.q    = 0.0
    cmd.dq   = 3.14*queryGearRatio(MotorType.A1)
    cmd.kp   = 0.0
    cmd.kd   = 2
    cmd.tau  = 0.0
    serial.sendRecv(cmd, data)
    print('\n')
    print("q: " + str(data.q))
    print("dq: " + str(data.dq))
    print("temp: " + str(data.temp))
    print("merror: " + str(data.merror))
    print('\n')
    time.sleep(0.5) # 2s

for i in range(3):
    serial.sendRecv(scmd, sdata)
