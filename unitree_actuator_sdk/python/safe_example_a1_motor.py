import time
import sys
sys.path.append('../lib')
from unitree_actuator_sdk import *


serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()

scmd = MotorCmd()
sdata = MotorData()
sdata.motorType = MotorType.A1
scmd.motorType = MotorType.A1
scmd.mode = queryMotorMode(MotorType.A1,MotorMode.BRAKE)
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

