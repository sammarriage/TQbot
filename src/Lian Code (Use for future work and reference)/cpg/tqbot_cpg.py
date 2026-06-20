# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Authors: Yunlong Lian, PhD student, Joe Ingham, MEng
# Date: 09-12-2021
# ----------------------------------------------------------------
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
import src.gaits.cpg_configuration as cpg_configuration
from src.cpg.basic_cpg import basicCpg


class tqbotCpg(basicCpg):
    def __init__(self, cpg_config: cpg_configuration.cpg_config) ->None :
        """
        cpg_config: cpg parameters settings
        """   
        super().__init__(cpg_config)

    #Allows the CPG config to changed midrun -- Joe Added
    def set_config(self, new_config: cpg_configuration.cpg_config) ->None :
        self.name = new_config.name
        # Oscillator numbers/timestep shouldn't be changed
        # change frequency
        self.v_stance = new_config.speed_stance
        self.v_swing = new_config.speed_swing

        # Update the current oscillators desired values: amplitude and offsets
        for i in range(self.numbers):
            self.osc[i].change_osc_vals(new_config.st_amplitude[i], new_config.sw_amplitude[i],new_config.desired_offset[i])

        # Dont update theta/phi, they should change smoothly on there own
        # change coupling matrix and phase lag matrix
        self.coupling_w = new_config.coupling_weights
        self.coupling_phi = new_config.phase_lag_matrix
    
    # Returns the curent CPGs oscillator desired amplitudes
    def get_amplitudes(self):
        stance_amp_list = []
        swing_amp_list = []
        for i in range(self.numbers):
            st, sw = self.osc[i].get_desired_amplitudes()
            stance_amp_list.append(st)
            swing_amp_list.append(sw)
        return stance_amp_list, swing_amp_list

    def set_amplitude(self, amp_st, amp_sw, osc_index):
        """set the amplitude of a single oscillator
        Args:
            amp (float): desired amplitude
            osc_index (_type_): the index of an oscillator
        """
        self.osc[osc_index].change_osc_amp(amp_st, amp_sw)

    # Allows the CPG amplitudes to be changed
    def set_amplitudes(self, new_stance_amps, new_swing_amps):
        for i in range(self.numbers):
            self.osc[i].change_osc_amp(new_stance_amps[i],new_swing_amps[i])

    # Get amplitude of specific oscillator
    def get_specific_amplitude(self, osc_num):
        
        return self.osc[osc_num].get_desired_amplitudes()

    # Sets the amplitude of a specific oscillator
    def set_specific_amplitude(self, new_stance_amp, new_swing_amp, osc_num):
        self.osc[osc_num].change_osc_amp(new_stance_amp, new_swing_amp)

    # Returns the curent CPGs oscillator desired offsets
    def get_offsets(self):
        offset_list = [0 for i in range(self.numbers)]
        for i in range(self.numbers):
            offset_list[i] = self.osc[i].get_desired_offset()
        return offset_list

    # Allows the CPG offsets to be changes
    def set_offset(self, osc_num, new_offset):
        self.osc[osc_num].set_osc_off(new_offset)
    
    def set_offsets(self, new_offsets):
        for i in range(self.numbers):
            self.osc[i].set_osc_off(new_offsets[i])

    # Get amplitude of specific oscillator
    def get_specific_offset(self, osc_num):
        return self.osc[osc_num].get_desired_offset()

    # Sets the amplitude of a specific oscillator
    def set_specific_offset(self, new_offset, osc_num):
        self.osc[osc_num].set_osc_off(new_offset)

    def get_stance_swing_speed(self):
        return self.v_stance, self.v_swing
            
    # Allows the CPG stance and swing speeds to be changed (also lets the frequency be changed)
    def set_stance_swing_speed(self, stance_speed, swing_speed):
        if type(stance_speed) == list:
            self.v_stance = stance_speed
        elif type(stance_speed) == float or type(stance_speed) == int:
            self.v_stance = [stance_speed for i in range(self.numbers)]
        else:
            raise TypeError("Error: the type of the set stance speed is wrong: should be list or int or float type")
        
        if type(swing_speed) == list:
            self.v_swing = swing_speed
        elif type(swing_speed) == float or type(stance_speed) == int:
            self.v_swing = [swing_speed for i in range(self.numbers)]
        else:
            raise TypeError("Error: the type of the set swing speed is wrong: should be list or int or float type")

    # Allows the CPG coupling to be changed
    def set_coupling(self, weights, phase_lags):
        self.coupling_w = weights
        self.coupling_phi = phase_lags

    def set_connection(self, new_weights):
        self.coupling_w = new_weights
    
    # Returns the phase-lag vector of the cpg config
    def get_phase_vec(self):
        return self.phase_vec

    # Edits the phase-lag matrix 
    def set_phase_specific(self, index, new_phase):
        # Get current phase vec
        phase_vec = self.phase_vec

        # Cahnge specific phase index
        phase_vec[index] = new_phase

        # Recreate the phase matrix
        new_phase_matrix = self.config.create_phase_matrix(phase_vec)
        
        # Set the new phase matrix 
        self.coupling_phi = new_phase_matrix
        
    def get_phase_specific(self, index):
        return self.phase_vec[index]

