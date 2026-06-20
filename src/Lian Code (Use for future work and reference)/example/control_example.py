# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author    : Yunlong Lian, PhD students
# File      : control_example.py
# Date      : 02-May-2023
# Version:  : 1.0.0
# ----------------------------------------------------------------
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

import time
from src.basic.motor_Lian_Old import Motor

if __name__=='__main__':
    motor = Motor(b'/dev/ttyUSB0', 0)
    # motor.safety_rotation()
    # motor.position_control(0)
    # time.sleep(1)
    control_f = 1000    # in Hz
    angle = 0
    # angle_step = 0.0174532 * 9.1 / control_f           # 1 deg/s
    angle_step = 0.105*2 * 9.1 / control_f  # 12 deg/s
    start_t = time.perf_counter()
    for i in range(2000):
        last_time = time.perf_counter()
        motor.position_control_send(angle)
        angle += angle_step
        while time.perf_counter() - last_time < 0.001:
            print("waiting for next seconds...")
    end_t = time.perf_counter()
    print("overall time is: ", end_t - start_t)
    print("target position is: ", angle)
    motor.stop_motor()