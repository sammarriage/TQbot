# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author: Yunlong Lian, PhD students
# Date: 07-02-2023
# ----------------------------------------------------------------
# CPG parameters
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from src.cpg.tqbot_cpg import tqbotCpg
from src.gaits.cpg_configuration import cpg_config
import numpy as np

# default constant variables
OSCILLATOR_NUMBERS = 16
CW_VALUE = 4
TIME_STEP = 1/60
CONSTANT_AR = 20
CONSTANT_AX = 20

# Default parameters
INITIAL_AMPLITUDE = np.zeros(OSCILLATOR_NUMBERS)
INITIAL_OFFSET = np.zeros(OSCILLATOR_NUMBERS)
INITIAL_PHASE = np.zeros(OSCILLATOR_NUMBERS)

# Different gaits parameters, 15 oscillators

# 1. phase lag
# BOUND_VECTOR = [0, 0, 0, 0, np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, 0, np.pi, 0, np.pi, np.pi/2, -np.pi/2, np.pi/2, -np.pi/2]
BOUND_VECTOR = [0, 0, np.pi/2, 0, 0, np.pi, 0, np.pi, np.pi*3/2, np.pi/2, np.pi*3/2, np.pi/2, 0, np.pi, 0, np.pi]

#2. frequency and 3. amplitude
SPEED_STANCE = [1 for i in range(OSCILLATOR_NUMBERS)]
SPEED_SWING = [1 for i in range(OSCILLATOR_NUMBERS)] 
# SWING_AMPLITUDE = [0, 0, 0.0873*2, 0, 0.025, 0.025, 0.025, 0.025, 0.35, 0.35, 0.35, 0.35, 0.2, 0.2, 0.2, 0.2] 
# STANCE_AMPLITUDE = [0, 0, 0.0873*2, 0, 0.025, 0.025, 0.025, 0.025, 0.35, 0.35, 0.35, 0.35, -0.1, -0.1, -0.1, -0.1] 
SWING_AMPLITUDE = [0, 0, 0.0873*2, 0, 0.025, 0.025, 0.025, 0.025, 0.44, 0.35, 0.44, 0.35, 0.262, 0.262, 0.262, 0.262] 
STANCE_AMPLITUDE = [0, 0, 0.0873*2, 0, 0.025, 0.025, 0.025, 0.025, 0.44, 0.35, 0.44, 0.35, -0.1, -0.1, -0.1, -0.1]
# 4. offset
DESIRED_OFFSET = [0, 0, 0, 0, 0.05, 0.05, 0.05, 0.05, 0.2, -0.2, 0.2, -0.2, -0.1, -0.1, -0.1, -0.1]

# Coupling weights, to determine connection structure of the CPG
FULL_CONNECTION_COUPLING_WEIGHTS = [[CW_VALUE if i != j else 0 for j in range(OSCILLATOR_NUMBERS)] for i in range(OSCILLATOR_NUMBERS)]
COUPLING_WEIGHTS = [[0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],    # reference oscillator has a connection with roll oscillator
                    [CW_VALUE, 0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE],  # 1 roll
                    [0, CW_VALUE, 0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE],         # 2 pitch
                    [0, CW_VALUE, CW_VALUE, 0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE],         # 3 yaw
                    [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0],                 # RF shoulder
                    [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0, 0],                 # RR shoulder
                    [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0],                 # LF shoulder
                    [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0],                 # LR shoulder
                    [0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0],          # RF hip
                    [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0],          # RR hip
                    [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0],          # LF hip
                    [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, CW_VALUE],          # LR hip
                    [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0],                 # RF knee
                    [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0, 0],                 # RR knee
                    [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0],                 # LF knee
                    [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0]]                 # LR knee

bound_parameters = cpg_config("bound", OSCILLATOR_NUMBERS, CONSTANT_AR, CONSTANT_AX, TIME_STEP, 
                             COUPLING_WEIGHTS, BOUND_VECTOR, SPEED_STANCE, SPEED_SWING, 
                             STANCE_AMPLITUDE, SWING_AMPLITUDE, DESIRED_OFFSET, INITIAL_AMPLITUDE, INITIAL_OFFSET, INITIAL_PHASE)

if __name__ == '__main__':
    # initialize CPG for test
    test_cpg = tqbotCpg(bound_parameters)
    theta_s = [[] for i in range(test_cpg.numbers)]
    r_s = [[] for i in range(test_cpg.numbers)]
    phi_s = [[] for i in range(test_cpg.numbers)]
    
    for i in range(600):
        test_cpg.update_x()
        test_cpg.update_r()
        phi_pos = test_cpg.update_phi()
        theta_v = test_cpg.update_setpoints()
        
        for i in range(test_cpg.numbers):
            r_s[i].append(test_cpg.amp_out[i])
            phi_s[i].append(test_cpg.phi[i])

        for i in range(len(theta_v)):
            theta_s[i].append(theta_v[i])

    # plot spine, shoulder, hip and knee joints results
    label = [['θ 4: Right Front', 'θ 5: Right Rear', 'θ 6: Left Front', 'θ 7: Left Rear'],
             ['θ 8: Right Front', 'θ 9: Right Rear', 'θ 10: Left Front', 'θ 11: Left Rear'],
             ['θ 12: Right Front', 'θ 13: Right Rear', 'θ 14: Left Front', 'θ 15: Left Rear']]
    title = ['Shoulder Joint', 'Hip Joint', 'Knee Joint']
    # test_cpg.plot_all_results(theta_s[1:4], theta_s[4:8], theta_s[8:12], theta_s[12:16], 411, label, title)
    test_cpg.plot_all_results_2(theta_s[1:4], theta_s[4:8], theta_s[8:12], theta_s[12:16], 411, label, title)
    label = [['Shoulder1', 'Hip5', 'Knee9'],
             ['Shoulder2', 'Hip6', 'Knee10'],
             ['Shoulder3', 'Hip7', 'Knee11'],
             ['Shoulder4', 'Hip8', 'Knee12']]
    title = ['right front', 'right rear', 'left front', 'left rear']
    # print(len(theta_s[1:10:4]))
    # test_cpg.plot_each_leg(theta_s[4:13:4], theta_s[5:14:4], theta_s[6:15:4], theta_s[7:16:4], 411, label, title)
    # [0-15] RF: 4, 8, 12
    print("Done")