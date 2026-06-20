# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author    : Yunlong Lian, PhD students
# File      : tqbot_control.py
# Date      : 16-Jul-2024
# Version:  : 
# ----------------------------------------------------------------
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from src.basic.motor_Lian_Old import Motor
from src.cpg.tqbot_cpg import tqbotCpg
from src.gaits.trot_parameters import trot_parameters
from typing import List

class tqbotControl():
    def __init__(self):
        motors_info = [(b'/dev/ttyUSB0', 0), (b'/dev/ttyUSB0', 1), (b'/dev/ttyUSB0', 2),    # Leg1, Right Front
                       (b'/dev/ttyUSB1', 0), (b'/dev/ttyUSB1', 1), (b'/dev/ttyUSB1', 2),    # Leg2, Right Rear
                       (b'/dev/ttyUSB2', 0), (b'/dev/ttyUSB2', 1), (b'/dev/ttyUSB2', 2),    # Leg3, Left Front
                       (b'/dev/ttyUSB3', 0), (b'/dev/ttyUSB3', 1), (b'/dev/ttyUSB3', 2),    # Leg4, Left Rear
                       (b'/dev/ttyUSB4', 0), (b'/dev/ttyUSB4', 1),  # Spine Left
                       (b'/dev/ttyUSB5', 0), (b'/dev/ttyUSB5', 1)]  # Spine Right

        self.motors: List[Motor] = [Motor(info[0], info[1]) for info in motors_info]
        self.cpg = tqbotCpg(trot_parameters)

    def _all_send(self, angles, kp=0.2, kw=3):
        for i in range(len(self.motors)):
            self.motors[i].position_control_send(angles[i], kp, kw)
