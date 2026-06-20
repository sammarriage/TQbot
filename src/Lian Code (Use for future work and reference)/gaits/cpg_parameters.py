# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author: Yunlong Lian, Tianyuan Wang, PhD students
# Date: 14-07-2022
# ----------------------------------------------------------------
# CPG parameters

import numpy as np

# default constant variables
OSCILLATOR_NUMBERS = 16
CW_VALUE = 4
TIME_STEP = 1/60
CONSTANT_AR = 20
CONSTANT_AX = 20

# Default parameters
INITIAL_AMPLITUDE = [0 for i in range(OSCILLATOR_NUMBERS)]
INITIAL_OFFSET = [0 for i in range(OSCILLATOR_NUMBERS)]
INITIAL_PHASE = [0 for i in range(OSCILLATOR_NUMBERS)]

# Different gaits parameters, 15 oscillators

# 1. phase lag
WALK_VECTOR = [0, 0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi, -np.pi, -np.pi/2, 0, np.pi/2, -np.pi/2, 0, np.pi/2, np.pi]
TROT_VECTOR = [0, 0, 0, 0, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, -np.pi, 0, 0, -np.pi, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2]
PACE_VECTOR = [0, 0, 0, 0, -np.pi/2, -np.pi/2, np.pi/2, np.pi/2, -np.pi, -np.pi, 0, 0, -np.pi/2, -np.pi/2, np.pi/2, np.pi/2]
BOUND_VECTOR = [0, 0, 0, 0, np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, 0, np.pi, 0, np.pi, np.pi/2, -np.pi/2, np.pi/2, -np.pi/2]

# 2. walk frequency and 3. amplitude
WALK_SPEED_STANCE = [0.4 for i in range(OSCILLATOR_NUMBERS)]
WALK_SPEED_SWING = [1.2 for i in range(OSCILLATOR_NUMBERS)]
WALK_DESIRED_AMPLITUDE = [0, 0.1, 0, 0.1, 0.05, 0.05, 0.05, 0.05, 0.785, 0.785, 0.785, 0.785, 0.4, 0.4, 0.4, 0.4]

# other gaits 2. frequency and 3. amplitude
SPEED_STANCE = [0.7 for i in range(OSCILLATOR_NUMBERS)]
SPEED_SWING = [0.7 for i in range(OSCILLATOR_NUMBERS)]
DESIRED_AMPLITUDE = [0, 0.1, 0, 0.1, 0.05, 0.05, 0.05, 0.05, 0.785, 0.785, 0.785, 0.785, 0.4, 0.4, 0.4, 0.4]  

# 4. offset
DESIRED_OFFSET = [0, 0, 0, 0, 0.1, 0.1, 0.1, 0.1, 0, 0, 0, 0, 0, 0, 0, 0]

# define coupling weights
# Coupling weights, to determine connection structure of the CPG
# full connection
FULL_CONNECTION_COUPLING_WEIGHTS = [[CW_VALUE if i != j else 0 for j in range(OSCILLATOR_NUMBERS)] for i in range(OSCILLATOR_NUMBERS)]

COUPLING_WEIGHTS = [[0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],    # reference oscillator has a connection with roll oscillator
                    [CW_VALUE, 0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, 0],  # roll
                    [0, CW_VALUE, 0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, 0],         # pitch
                    [0, CW_VALUE, CW_VALUE, 0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, 0],         # yaw
                    [0, CW_VALUE*4, CW_VALUE*4, CW_VALUE*4, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0],                 # RF shoulder 
                    [0, CW_VALUE*4, CW_VALUE*4, CW_VALUE*4, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0, 0],                 # RR shoulder
                    [0, CW_VALUE*4, CW_VALUE*4, CW_VALUE*4, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0],                 # LF shoulder
                    [0, CW_VALUE*4, CW_VALUE*4, CW_VALUE*4, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0],                 # LR shoulder
                    [0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, CW_VALUE, CW_VALUE, 0, CW_VALUE, 0, 0, 0],                       # RF hip
                    [0, 0, 0, 0, 0, CW_VALUE, 0, 0, CW_VALUE, 0, 0, CW_VALUE, 0, CW_VALUE, 0, 0],                       # RR hip
                    [0, 0, 0, 0, 0, 0, CW_VALUE, 0, CW_VALUE, 0, 0, CW_VALUE, 0, 0, CW_VALUE, 0],                       # LF hip
                    [0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, CW_VALUE],             # LR hip
                    [0, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, CW_VALUE, CW_VALUE, 0],                              # RF knee
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, CW_VALUE, 0, 0, CW_VALUE],                              # RR knee
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, CW_VALUE, 0 ,0, CW_VALUE],                              # LF knee
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,CW_VALUE, 0, CW_VALUE, CW_VALUE, 0]]                               # LR knee
# partial connection
B_CONNECTION_WEIGHTS = [[0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],    # reference oscillator has a connection with roll oscillator
                        [CW_VALUE, 0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE],  # 1 roll
                        [0, CW_VALUE, 0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE],         # 2 pitch
                        [0, CW_VALUE, CW_VALUE, 0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE],         # 3 yaw
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, CW_VALUE, CW_VALUE, 0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0],                   # 4 RF shoulder
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, CW_VALUE, 0, CW_VALUE, 0, 0, 0, 0, 0, 0],                   # 5 RR shoulder
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, CW_VALUE, 0, 0, CW_VALUE, 0, 0, 0, 0, 0],                   # 6 LF shoulder
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, CW_VALUE, CW_VALUE, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0],                   # 7 LR shoulder
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, CW_VALUE, CW_VALUE, 0, CW_VALUE, 0, 0, 0],            # 8 RF hip
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, CW_VALUE, 0, 0, CW_VALUE, 0, 0, CW_VALUE, 0, CW_VALUE, 0, 0],            # 9 RR hip
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, CW_VALUE, 0, CW_VALUE, 0, 0, CW_VALUE, 0, 0, CW_VALUE, 0],            # 10 LF hip
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, CW_VALUE, 0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, CW_VALUE],  # 11 LR hip
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, CW_VALUE, CW_VALUE, 0],                   # 12 RF knee
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, CW_VALUE, 0, 0, CW_VALUE],                   # 13 RR knee
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, CW_VALUE, 0, 0, CW_VALUE],                   # 14 LF knee
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, CW_VALUE, CW_VALUE, 0]]                   # 15 LR knee

C_CONNECTION_WEIGHTS = [[0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],    # reference oscillator has a connection with roll oscillator
                        [CW_VALUE, 0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, 0],  # roll
                        [0, CW_VALUE, 0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, 0],         # pitch
                        [0, CW_VALUE, CW_VALUE, 0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, 0],         # yaw
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, CW_VALUE, CW_VALUE, 0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0],                   # 4 RF shoulder
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, CW_VALUE, 0, CW_VALUE, 0, 0, 0, 0, 0, 0],                   # 5 RR shoulder
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, CW_VALUE, 0, 0, CW_VALUE, 0, 0, 0, 0, 0],                   # 6 LF shoulder
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, CW_VALUE, CW_VALUE, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0],                   # 7 LR shoulder
                        [0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, CW_VALUE, CW_VALUE, 0, CW_VALUE, 0, 0, 0],                       # RF hip
                        [0, 0, 0, 0, 0, CW_VALUE, 0, 0, CW_VALUE, 0, 0, CW_VALUE, 0, CW_VALUE, 0, 0],                       # RR hip
                        [0, 0, 0, 0, 0, 0, CW_VALUE, 0, CW_VALUE, 0, 0, CW_VALUE, 0, 0, CW_VALUE, 0],                       # LF hip
                        [0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, CW_VALUE],             # LR hip
                        [0, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, CW_VALUE, CW_VALUE, 0],                              # RF knee
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, CW_VALUE, 0, 0, CW_VALUE],                              # RR knee
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, CW_VALUE, 0, 0, CW_VALUE],                              # LF knee
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, CW_VALUE, CW_VALUE, 0]]                               # LR knee

D_CONNECTION_WEIGHTS = [[0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],    # reference oscillator has a connection with roll oscillator
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
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, 0, CW_VALUE],       # LR hip
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0],         # RF knee
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0, 0],         # RR knee
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0],         # LF knee
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0]]         # LR knee

E_CONNECTION_WEIGHTS = [[0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],    # reference oscillator has a connection with roll oscillator
                        [CW_VALUE, 0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, 0],  # roll
                        [0, CW_VALUE, 0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, 0],         # pitch
                        [0, CW_VALUE, CW_VALUE, 0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, 0],         # yaw
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0],                 # RF shoulder
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0, 0],                 # RR shoulder
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0],                 # LF shoulder
                        [0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0],                 # LR shoulder
                        [0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0],          # RF hip
                        [0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0],          # RR hip
                        [0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0],          # LF hip
                        [0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, 0, CW_VALUE],       # LR hip
                        [0, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0],         # RF knee
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0, 0],         # RR knee
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0],         # LF knee
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0]]         # LR knee

# used for plotting
PLOT_DESIRED_AMPLITUDE = [0, 0.1, 0, 0.1, 0.05, 0.05, 0.05, 0.05, 0.4, 0.4, 0.4, 0.4, 0.2, 0.2, 0.2, 0.2]
PLOT_DESIRED_OFFSET = [0, 0, 0, 0, 0.025, 0.025, 0.025, 0.025, 1, 1, 1, 1, 1.745, 1.745, 1.745, 1.745]
PLOT_INITIAL_OFFSET = [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1.745, 1.745, 1.745, 1.745]
PLOT_INITIAL_AMPLITUDE = [0 for i in range(OSCILLATOR_NUMBERS)]
PLOT_INITIAL_PHASE = [0 for i in range(OSCILLATOR_NUMBERS)]

if __name__ == '__main__':
    print(FULL_CONNECTION_COUPLING_WEIGHTS)