# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author: Yunlong Lian, PhD student
# Date: 09-12-2021
# ----------------------------------------------------------------
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from src.cpg.differential_funs import differentialEq

class oscillator():
    def __init__(self, step_size, ar, ax, R_st, R_sw, X, r_0, x_0):
        self.amp_st = differentialEq(ar, R_st, step_size, r_0)
        self.amp_sw = differentialEq(ar, R_sw, step_size, r_0)
        self.offset = differentialEq(ax, X, step_size, x_0)
        self.offset_out = 0
        
    def update_r(self):
        st_pos = self.amp_st.update_amp_3()
        sw_pos = self.amp_sw.update_amp_3()
        return st_pos, sw_pos

    def update_x(self):
        self.offset_out = self.offset.update_amp_3()
        return self.offset_out
    
    #----JOE----
    #Returns current R value
    def get_desired_amplitudes(self):
        return self.amp_st.R, self.amp_sw.R

    #Returns current X Value 
    def get_desired_offset(self):
        return self.offset.R
   
    #Updates Oscillators values - Desired Amplitude, Desired Offset
    def change_osc_vals(self, new_amp_st, new_amp_sw, new_off):
        self.amp_st.R = new_amp_st
        self.amp_sw.R = new_amp_sw
        self.offset.R = new_off

    #Update oscillators amplitude
    def change_osc_amp(self, new_amp_st, new_amp_sw):
        self.amp_st.R = new_amp_st
        self.amp_sw.R = new_amp_sw
    
        #Update oscillators offset
    def set_osc_off(self, new_off):
        self.offset.R = new_off




