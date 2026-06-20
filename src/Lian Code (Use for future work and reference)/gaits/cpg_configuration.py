# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author: Yunlong Lian, PhD student
# Date: 23-11-2022
# ----------------------------------------------------------------
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from src.gaits.cpg_parameters import *

class cpg_config():
    def __init__(self, NAME: str, OSCILLATOR_NUMBERS: int, CONSTANT_AR: int, CONSTANT_AX: int, TIME_STEP: float, 
                 COUPLING_WEIGHTS: list, PHASE_VECTOR: list, SPEED_STANCE: list, SPEED_SWING: list, 
                 STANCE_AMPLITUDE: list, SWING_AMPLITUDE: list, DESIRED_OFFSET: list, 
                 INITIAL_AMPLITUDE: list, INITIAL_OFFSET: list, INITIAL_PHASE: list) -> None:
        self.name = NAME
        self.oscillator_number = OSCILLATOR_NUMBERS

        #So that the phase lag can be edited
        self.phase_lag_vector = PHASE_VECTOR

        # convert all matrix to np.array()
        self.coupling_weights = np.array(COUPLING_WEIGHTS)
        self.phase_lag_matrix = self.create_phase_matrix(PHASE_VECTOR)
        self.speed_stance = np.array(SPEED_STANCE)
        self.speed_swing = np.array(SPEED_SWING)
        self.st_amplitude = np.array(STANCE_AMPLITUDE)
        self.sw_amplitude = np.array(SWING_AMPLITUDE)
        self.desired_offset = np.array(DESIRED_OFFSET)

        # self.coupling_weights = COUPLING_WEIGHTS
        # self.phase_lag_matrix = self.create_phase_matrix(PHASE_VECTOR)
        # self.speed_stance = SPEED_STANCE
        # self.speed_swing = SPEED_SWING
        # self.st_amplitude = STANCE_AMPLITUDE
        # self.sw_amplitude = SWING_AMPLITUDE
        # self.desired_offset = DESIRED_OFFSET
        self.initial_amplitude = INITIAL_AMPLITUDE
        self.initial_offset = INITIAL_OFFSET
        self.initial_phase = INITIAL_PHASE

        # define constant variables
        self.ar = CONSTANT_AR
        self.ax = CONSTANT_AX
        self.time_step = TIME_STEP
    
    def create_phase_matrix(self, initial_vector):
        assert self.oscillator_number == len(initial_vector)
        matrix = np.zeros((self.oscillator_number, self.oscillator_number))
        matrix[0] = initial_vector
        # print(len(matrix[0]))
        for i in range(1, len(matrix)):
            for j in range(i+1, len(matrix[i])):
                matrix[i][j] = initial_vector[j] - initial_vector[i]
        matrix = -matrix.T + matrix
        return matrix

# define parameters

# walk_parameters = cpg_config("walk", OSCILLATOR_NUMBERS, CONSTANT_AR, CONSTANT_AX, TIME_STEP, 
#                              COUPLING_WEIGHTS3, WALK_VECTOR, WALK_SPEED_STANCE, WALK_SPEED_SWING, 
#                              WALK_DESIRED_AMPLITUDE, DESIRED_OFFSET, INITIAL_AMPLITUDE, INITIAL_OFFSET, INITIAL_PHASE)

# trot_parameters = cpg_config("trot", OSCILLATOR_NUMBERS, CONSTANT_AR, CONSTANT_AX, TIME_STEP, 
#                              COUPLING_WEIGHTS3, TROT_VECTOR, SPEED_STANCE, SPEED_SWING, 
#                              DESIRED_AMPLITUDE, DESIRED_OFFSET, INITIAL_AMPLITUDE, INITIAL_OFFSET, INITIAL_PHASE)

# pace_parameters = cpg_config("pace", OSCILLATOR_NUMBERS, CONSTANT_AR, CONSTANT_AX, TIME_STEP, 
#                              COUPLING_WEIGHTS1, PACE_VECTOR, SPEED_STANCE, SPEED_SWING, 
#                              DESIRED_AMPLITUDE, DESIRED_OFFSET, INITIAL_AMPLITUDE, INITIAL_OFFSET, INITIAL_PHASE)

# bound_parameters = cpg_config("bound", OSCILLATOR_NUMBERS, CONSTANT_AR, CONSTANT_AX, TIME_STEP, 
#                               COUPLING_WEIGHTS1, BOUND_VECTOR, SPEED_STANCE, SPEED_SWING, 
#                              DESIRED_AMPLITUDE, DESIRED_OFFSET, INITIAL_AMPLITUDE, INITIAL_OFFSET, INITIAL_PHASE)


# FULL_CONNECTION_COUPLING_WEIGHTS = [[CW_VALUE if i!=j else 0 for j in range(15)] for i in range(15)]

# plot_trot_parameters = cpg_config("plot_trot", OSCILLATOR_NUMBERS, CONSTANT_AR, CONSTANT_AX, TIME_STEP, 
#                                   COUPLING_WEIGHTS3, TROT_VECTOR, SPEED_STANCE, SPEED_SWING, 
#                                   PLOT_DESIRED_AMPLITUDE, PLOT_DESIRED_AMPLITUDE, PLOT_DESIRED_OFFSET, PLOT_INITIAL_AMPLITUDE, 
#                                   PLOT_INITIAL_OFFSET, PLOT_INITIAL_PHASE)

# plot_walk_parameters = cpg_config("plot_walk", OSCILLATOR_NUMBERS, CONSTANT_AR, CONSTANT_AX, TIME_STEP, 
#                                   COUPLING_WEIGHTS3, WALK_VECTOR, SPEED_STANCE, WALK_SPEED_SWING, 
#                                   PLOT_DESIRED_AMPLITUDE, PLOT_DESIRED_OFFSET, PLOT_INITIAL_AMPLITUDE, 
#                                   PLOT_INITIAL_OFFSET, PLOT_INITIAL_PHASE)