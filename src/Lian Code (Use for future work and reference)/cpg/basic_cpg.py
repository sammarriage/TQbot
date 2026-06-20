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

import matplotlib.pyplot as plt
import numpy as np
import src.gaits.cpg_configuration as cpg_configuration
from src.cpg.phase_oscillator import oscillator
from typing import List
import os


class basicCpg():
    def __init__(self, cpg_config: cpg_configuration.cpg_config) -> None:
        """
        cpg_config: cpg parameters settings
        """
        self.name = getattr(cpg_config, "name")
        self.config = cpg_config
        self.numbers = cpg_config.oscillator_number
        self.stepsize = cpg_config.time_step
        self.v_stance = cpg_config.speed_stance
        self.v_swing = cpg_config.speed_swing

        # NEW amplitude vectors
        self.amp_st = cpg_config.st_amplitude
        self.amp_sw = cpg_config.sw_amplitude
        self.amp_out = np.zeros(self.numbers)
        self.osc: List[oscillator] = [n for n in range(self.numbers)]

        for i in range(self.numbers):
            self.osc[i] = oscillator(self.stepsize, cpg_config.ar, cpg_config.ax, 
                                     cpg_config.st_amplitude[i], 
                                     cpg_config.sw_amplitude[i], 
                                     cpg_config.desired_offset[i], 
                                     cpg_config.initial_amplitude[i],
                                     cpg_config.initial_offset[i])
        self.theta = np.zeros(self.numbers)

        # state variable
        self.phi = cpg_config.initial_phase
        self.coupling_w = cpg_config.coupling_weights
        self.coupling_phi = cpg_config.phase_lag_matrix

        self.phase_vec = cpg_config.phase_lag_vector

        # print("oscillators: ",self.osc)
        # print("theta: ", self.theta)
        # print("phi: ", self.phi)
        # print("frequency: ", self.v)
        # print("coupling_phi: ", self.coupling_phi)

    def _manually_contact_detection(self, last_hip_angles, current_hip_angles):
        contact_km = [1, 1, 1, 1]
        for i in range(4):   # used to determine if feet has touched the ground
            if current_hip_angles[i] - last_hip_angles[i] > 0:  # swing phase
                contact_km[i] = 0
                # print("swing")
            else:  # stance phase
                contact_km[i] = 1
                # print("stance")
        return contact_km

    def _which_leg(self, joint_index):
        i = joint_index
        if i == self.numbers - 12 or i == self.numbers - 8 or i == self.numbers - 4:
            m = 0    # 4, 8, 12
        elif i == self.numbers - 11 or i == self.numbers - 7 or i == self.numbers - 3:
            m = 1    # 5, 9, 13
        elif i == self.numbers - 10 or i == self.numbers - 6 or i == self.numbers - 2:
            m = 2    # 6, 10, 14
        elif i == self.numbers - 9 or i == self.numbers - 5 or i == self.numbers - 1:
            m = 3    # 7, 11, 15
        elif i <= self.numbers - 13:  # 16-13 = 3, spine
            m = 0
        else:
            print("joint number is ", i)
            raise ValueError("Joint number is incorrect!")
        return m

    def hip_feedback(self, phi_offest):
        feed_i = []
        rf_hip_index = self.numbers - 8     # 16-8=8
        lr_hip_index = self.numbers - 5     # 16-5=11
        for phi in self.phi[rf_hip_index:lr_hip_index + 1]:
            # print("phi is ", phi)
            new_phi = phi + phi_offest
            first_v = self.first_order_derivative(new_phi)
            second_v = self.second_order_derivative(new_phi)
            if first_v == 0:
                if second_v <= 0:
                    feed_i.append(1)
                else:
                    feed_i.append(0)
            else:
                feed_i.append(self.piecewise_fun(first_v))
        return np.array(feed_i)

    def first_order_derivative(self, phi_i):  # First Order Derivative
        return -np.sin(phi_i)

    def second_order_derivative(self, phi_i):
        return -np.cos(phi_i)

    def piecewise_fun(self, first_order_d):
        return (- first_order_d / abs(first_order_d) + 1) / 2

    def update_r(self, phi_offset=0):
        fb = self.hip_feedback(phi_offset)
        for i in range(self.numbers):
            r_st, r_sw = self.osc[i].update_r()
            leg_num = self._which_leg(i)
            self.amp_out[i] = fb[leg_num] * (r_st - r_sw) + r_sw
        return True

    def update_x(self):
        for osc in self.osc:
            osc.update_x()
        return True

    def update_phi(self, phi_offest=0):
        """
        before update phi, the "r" and "x" should be updated firstly
        """

        # dy_phi = [0 for i in range(self.numbers)]
        # sum = [0 for i in range(self.numbers)]
        dy_phi = np.zeros(self.numbers)
        sum = np.zeros(self.numbers)
        fb = self.hip_feedback(phi_offest)
        # calculate all dy_phi
        # oscillator 0 is spine joint
        # print("Start update phi: ")
        for i in range(self.numbers):
            for j in range(self.numbers):
                sum[i] += self.coupling_w[i][j] * np.sin(self.phi[j] - self.phi[i] - self.coupling_phi[i][j])  # without times r
                # print("sum[{0}] : {1}".format(i, sum[i]))
                # print("sum: ", sum[i])

            # independently control of the frequency of stance and swing phase
            leg_num = self._which_leg(i)
            v_i = fb[leg_num] * (self.v_stance[i] - self.v_swing[i]) + self.v_swing[i]
            dy_phi[i] = 2 * np.pi * v_i + sum[i]
            # print(i, ": ", dy_phi[i])

        # update all the phi
        self.phi += dy_phi * self.stepsize
        # for i in range(self.numbers):
        #     self.phi[i] += dy_phi[i] * self.stepsize
            # print("phi: ", self.phi[i])
        return self.phi

    def update_setpoints(self):
        """
        before update setpoints, we should update phi firstly
        """
        all_offset_out = np.array([self.osc[i].offset_out for i in range(self.numbers)])
        self.theta = all_offset_out + self.amp_out*np.cos(self.phi)
        # for i in range(self.numbers):
        #     self.theta[i] = self.osc[i].offset_out + self.amp_out[i] * np.cos(self.phi[i])
        return self.theta
    
    def get_curr_amp(self, osc_num):
        return self.osc[osc_num].amp_st.get_pos(), self.osc[osc_num].amp_sw.get_pos()
    
    def get_setpoint(self, osc_num):
        return self.theta[osc_num]

    @staticmethod
    def plot_all_results(spine, y1, y2, y3, num, label, title):
        l = len(y1[0])    # e.g. if y1[0] has 1200 data points, l = len(y1) = 1200
        t_interval = 1 / (10 ** (len(str(l)) - 2))  # this is 1/100
        # print(t_interval)
        # t_end = int(l * t_interval)    # 4
        t_end = l * t_interval
        # print(t_end)
        t = np.arange(0, t_end, t_interval)
        # print(l,':', t_end, ":", t_interval)
        plt.figure(figsize=(16, 8))
        plt.subplot(num + 0)
        plt.plot(t, [i+0.2 for i in spine[0]], color='green', label='θ 1: spine roll')
        plt.plot(t, [i for i in spine[1]], color='brown', label='θ 2: spine pitch')
        plt.plot(t, [i-0.2 for i in spine[2]], color='purple', label='θ 3: spine yaw')
        plt.axis([0, t_end, -0.35, 0.35])
        plt.yticks([])
        plt.xticks([])
        plt.legend(loc="right")
        
        plt.subplot(num + 1)
        plt.plot(t, [i + 0.125 for i in y1[0]], color='red', label=label[0][0])
        plt.plot(t, [i + 0.125 for i in y1[1]], color='blue', label=label[0][1])
        plt.plot(t, y1[2], color='orange', label=label[0][2])
        plt.plot(t, y1[3], color='black', label=label[0][3])
        plt.axis([0, t_end, 0, 0.3])
        plt.yticks([])
        plt.xticks([])
        # plt.title(title[0])
        plt.legend(loc="right")
        plt.subplot(num + 2)
        plt.plot(t, [i + 2 for i in y2[0]], color='red', label=label[1][0])
        plt.plot(t, [i + 2 for i in y2[1]], color='blue', label=label[1][1])
        plt.plot(t, [i + 1 for i in y2[2]], color='orange', label=label[1][2])
        plt.plot(t, [i + 1 for i in y2[3]], color='black', label=label[1][3])
        plt.axis([0, t_end, 0.25, 3])
        plt.yticks([])
        plt.xticks([])
        # plt.title(title[1])
        plt.legend(loc="right")
        plt.subplot(num + 3)
        plt.plot(t, [i + 2.5 for i in y3[0]], color='red', label=label[2][0])
        plt.plot(t, [i + 2.5 for i in y3[1]], color='blue', label=label[2][1])
        plt.plot(t, [i + 1.75 for i in y3[2]], color='orange', label=label[2][2])
        plt.plot(t, [i + 1.75 for i in y3[3]], color='black', label=label[2][3])
        plt.axis([0, t_end, 1.25, 3.2])
        plt.yticks([])
        # plt.title(title[2])
        plt.legend(loc="right")
        # plt.tight_layout()
        plt.show()

    @staticmethod
    def plot_all_results_2(spine, y1, y2, y3, num, label, title, step=60, savePath=None):
        # the function plots of the CPG results in 60 hz, which is the same as the simulation
        t_end = len(y1[0]) / step   # len(y1[0])=1200, t_end = 20
        t_interval = 1 / step
        # print(t_end)
        t = np.arange(0, t_end, t_interval)
        # print(len(t))
        # print(l,':', t_end, ":", t_interval)
        plt.figure(figsize=(16, 8))
        plt.subplot(num + 0)
        plt.plot(t, [i+0.2 for i in spine[0]], color='green', label='θ 1: spine roll')
        plt.plot(t, [i for i in spine[1]], color='brown', label='θ 2: spine pitch')
        plt.plot(t, [i-0.2 for i in spine[2]], color='purple', label='θ 3: spine yaw')
        plt.axis([0, t_end, -0.35, 0.35])
        plt.yticks([])
        plt.xticks([])
        plt.legend(loc="right")
        
        plt.subplot(num + 1)
        plt.plot(t, [i + 0.125 for i in y1[0]], color='red', label=label[0][0])
        plt.plot(t, [i + 0.125 for i in y1[1]], color='blue', label=label[0][1])
        plt.plot(t, y1[2], color='orange', label=label[0][2])
        plt.plot(t, y1[3], color='black', label=label[0][3])
        plt.axis([0, t_end, 0, 0.3])
        plt.yticks([])
        plt.xticks([])
        plt.legend(loc="right")
        plt.subplot(num + 2)
        plt.plot(t, [i + 2 for i in y2[0]], color='red', label=label[1][0])
        plt.plot(t, [i + 2 for i in y2[1]], color='blue', label=label[1][1])
        plt.plot(t, [i + 1 for i in y2[2]], color='orange', label=label[1][2])
        plt.plot(t, [i + 1 for i in y2[3]], color='black', label=label[1][3])
        plt.axis([0, t_end, 0.25, 3])
        plt.yticks([])
        plt.xticks([])
        plt.legend(loc="right")
        plt.subplot(num + 3)
        plt.plot(t, [i + 2.5 for i in y3[0]], color='red', label=label[2][0])
        plt.plot(t, [i + 2.5 for i in y3[1]], color='blue', label=label[2][1])
        plt.plot(t, [i + 1.75 for i in y3[2]], color='orange', label=label[2][2])
        plt.plot(t, [i + 1.75 for i in y3[3]], color='black', label=label[2][3])
        plt.axis([0, t_end, 1.25, 3.2])
        plt.yticks([])
        plt.xlabel("Time")
        plt.legend(loc="right")
        # plt.tight_layout()
        if savePath is not None:
            # Checks to make sure the results plots folders exists
            if os.path.exists(savePath):
                plt.savefig(savePath + title + ".png")
                print("Picture was saved")
            else:
                # If not - create the folder
                print(savePath, ' is not exists')
        plt.show()

    @staticmethod
    def plot_each_leg(y1, y2, y3, y4, num, label, title):
        l = len(y1[0])
        t_interval = 1 / (10 ** (len(str(l)) - 1))
        # print(t_interval)
        t_end = int(l * t_interval)
        # print(t_end)
        t = np.arange(0, t_end, t_interval)
        # print(l,':', t_end, ":", t_interval)
        plt.figure(figsize=(8, 6))

        # right front leg
        plt.subplot(num + 0)
        plt.plot(t, y1[0], color='red', label=label[0][0])
        plt.plot(t, y1[1], color='blue', label=label[0][1])
        plt.plot(t, y1[2], color='orange', label=label[0][2])
        plt.axis([0, t_end, -0.75, 0.6])
        # plt.title(title[0])
        plt.legend(loc="right")
        plt.subplot(num + 1)
        plt.plot(t, y2[0], color='red', label=label[1][0])
        plt.plot(t, y2[1], color='blue', label=label[1][1])
        plt.plot(t, y2[2], color='orange', label=label[1][2])
        plt.axis([0, t_end, -0.75, 0.6])
        # plt.title(title[1])
        plt.legend(loc="right")
        plt.subplot(num + 2)
        plt.plot(t, y3[0], color='red', label=label[2][0])
        plt.plot(t, y3[1], color='blue', label=label[2][1])
        plt.plot(t, y3[2], color='orange', label=label[2][2])
        plt.axis([0, t_end, -0.75, 0.6])
        # plt.title(title[2])
        plt.legend(loc="right")
        plt.subplot(num + 3)
        plt.plot(t, y4[0], color='red', label=label[3][0])
        plt.plot(t, y4[1], color='blue', label=label[3][1])
        plt.plot(t, y4[2], color='orange', label=label[3][2])
        plt.axis([0, t_end, -0.75, 0.6])
        # plt.title(title[2])
        plt.legend(loc="right")
        # plt.tight_layout()
        plt.show()