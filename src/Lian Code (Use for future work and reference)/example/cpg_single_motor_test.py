# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author    : Yunlong Lian, PhD students
# File      : cpg_single_motor_test.py
# Date      : 18-Jul-2024
# Version:  : 
# ----------------------------------------------------------------
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

import time
from src.cpg.tqbot_cpg import tqbotCpg
from src.gaits.trot_parameters import trot_parameters

if __name__ == "__main__":
    # initial motor and send stop command to get the current status
    # motor = Motor(b'/dev/ttyUSB0', 0)
    # motor.stop_motor()
    # get the current position
    # current_pos = motor.position

    # setup parameters
    control_f = 1/1000    # in Hz
    # initial CPG
    test_cpg = tqbotCpg(trot_parameters)
    angles = [[] for i in range(test_cpg.numbers)]
    r_s = [[] for i in range(test_cpg.numbers)]
    phi_s = [[] for i in range(test_cpg.numbers)]

    # calculate necessary information
    # interval = control_f / trot_parameters.time_step    # time_step = 1/60, interval = 60/1000
    # steps = 1/interval    # 16.6667
    interval = trot_parameters.time_step
    print("interval: ", interval)
    # print("steps: ", steps)
    # to simulate a running process
    init_time = time.perf_counter()
    for i in range(600):    # each step is 1/60 s
        start_t = time.perf_counter()
        test_cpg.update_x()
        test_cpg.update_r()
        phi_pos = test_cpg.update_phi()
        angles = test_cpg.update_setpoints()
        print("CPG calcuation time: ", time.perf_counter() - start_t)    # 0.0044, 0.00055

        # while (time.perf_counter() - start_t) < interval:    # each iteration is 16.6667 ms
        #     target_pos = current_pos + angles[10]*9.1
        #     motor.position_control_send(target_pos)
        #     print("waiting")
        #     pass

    # motor.stop_motor()
    print("total time: ", time.perf_counter()-init_time)