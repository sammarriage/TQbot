# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author    : Yunlong Lian, PhD students
# File      : spine_control.py
# Date      : 09-Aug-2023
# Version:  : 1.0.0
# ----------------------------------------------------------------
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

import numpy as np
from src.basic.quaternion import Quaternion

class SpineControl:
    def __init__(self):
        """
        Before starting to control the spine, make sure the spine is tensed
        """
        # T2 spine: 62.5mm = 0.0625
        self._R = np.array([0.0625, 0, 0.0805])  # R/U: Upper-Mid point  (Front point)
        self._Q = np.array([0.0625, 0.135, -0.068])  # Q: Lower-Left point
        self._P = np.array([0.0625, -0.135, -0.068])  # P: Lower-Right point
        self._r = np.array([[-0.0875, 0.135, 0.068],  # A1: Upper-Left      (Back Point)  Port: ttyUSB0, ID: 1
                            [-0.0875, -0.135, 0.068],  # A2: Upper-Right                   Port: ttyUSB1, ID: 1
                            [-0.0875, 0.135, -0.068],  # A3: Lower-Left                    Port: ttyUSB0, ID: 0
                            [-0.0875, -0.135, -0.068]])  # A4: Lower-Right                   Port: ttyUSB1, ID: 0
        self._roll = 0
        self._pitch = 0
        self._yaw = 0
        # real position: T1 spine
        # self._R = np.array([-0.05, 0, 0.068])         # R: Upper-Mid point  (Front point)
        # self._Q = np.array([-0.05, -0.135, -0.068])   # Q: Lower-Left point
        # self._P = np.array([-0.05, 0.135, -0.068])    # P: Lower-Right point
        # self._r = np.array([[0.05, -0.135, 0.068],    # A1: Upper-Left      (Back Point)  Port: ttyUSB0, ID: 1
        #                     [0.05, 0.135, 0.068],     # A2: Upper-Right                   Port: ttyUSB1, ID: 1
        #                     [0.05, -0.135, -0.068],   # A3: Lower-Left                    Port: ttyUSB0, ID: 0
        #                     [0.05, 0.135, -0.068]])   # A4: Lower-Right                   Port: ttyUSB1, ID: 0

        # Chrono definition, just for validating algorithm
        # self._R = np.array([-0.05, 0, 0.06])         # FR: front upper point
        # self._Q = np.array([-0.05, 0.135, -0.06])    # FDL: front down left
        # self._P = np.array([-0.05, -0.135, -0.06])   # FDR: front down right
        # self._r = np.array([[0.05, 0.135, 0.06],     # BUL: back upper left
        #                     [0.05, -0.135, 0.06],    # BUR
        #                     [0.05, 0.135, -0.06],    # BDL
        #                     [0.05, -0.135, -0.06]])  # BDR

        self._update_parameters()
        self.previous_wires_length = np.array([np.linalg.norm(v) for v in self._p])
        # print(self.previous_wires_length)    # initial length of each wire
        # self.P = self._P.reshape(3, 1)
        # self.R = self._R.reshape(3, 1)
        # self.Q = self._Q.reshape(3, 1)

    def get_angles(self):
        """
        return the current rotation angles of the spine in the order of roll, pitch, yaw
        """
        _rpy = np.array([self._roll, self._pitch, self._yaw])
        return _rpy

    def _update_parameters(self):
        # calculation
        self._p = np.array([self._R - self._r[0],        # R-A1
                            self._R - self._r[1],        # R-A2
                            self._Q - self._r[2],        # Q-A3
                            self._P - self._r[3]])       # P-A4

        self.w = np.cross(self._r, self._p)
        # self.w_t = self.w.T
        # self.wires_length = [np.linalg.norm(v) for v in self._p]
        # self.w_pinv = np.linalg.pinv(self.w_t)

    def _cal_tau(self):
        # self._update_parameters()
        n_matrix = self.w[:-1].T            # structural matrix
        w4 = self.w[-1].reshape(3, 1)       # redunant vector
        n_inv = -np.linalg.inv(n_matrix)                     # -N^-1 in formula (7)
        tmp_matrix = n_inv.dot(w4)
        # print(tmp_matrix)
        arbitrary_tau = np.array([[0.3, 0.3, 0.3]])  # set a positive value
        tau_n = tmp_matrix.dot(arbitrary_tau)         # formula (7)
        all_tau = np.append(tau_n, arbitrary_tau, axis=0)    # formula (8)
        return all_tau

    def _rotate(self, q: Quaternion, roll: float, pitch: float, yaw: float):
        # roll-pitch-yaw rotation sequence
        # rotate the vector q
        rotation_q = Quaternion.euler_to_Q(roll, pitch, yaw)
        q_rpy = rotation_q * q * rotation_q.conjugate()
        return q_rpy

    def cal_active_length(self, roll: float, pitch: float, yaw: float, ratio: float = 1):
        """
        Input the amount of spine rotation in roll, pitch and yaw orientation, and return each wires' length
        and length difference from the last wires' length. The model is built in right-hand coordinate and for front body.
        :param roll: in radian,
        :param pitch: in radian, (Upward is the positive direction)
        :param yaw: in radian, (turn left is positive direction)
        :param ratio:
        :return: wires' length, wires' difference length
        """
        # calculate wires' length before rotation
        # self.previous_wires_length = [np.linalg.norm(v) for v in self._p]
        # rotate and calculate wires' length
        # 1. Convert vectors to quaternion
        q_R = Quaternion(0, self._R[0], self._R[1], self._R[2])    # R
        q_P = Quaternion(0, self._P[0], self._P[1], self._P[2])    # P
        q_Q = Quaternion(0, self._Q[0], self._Q[1], self._Q[2])    # Q
        # print(q_Q)
        # 2. Rotate each wire
        new_q_R = self._rotate(q_R, roll, pitch, yaw)
        new_q_P = self._rotate(q_P, roll, pitch, yaw)
        new_q_Q = self._rotate(q_Q, roll, pitch, yaw)
        # record rotation angles
        self._roll += roll
        self._pitch += pitch
        self._yaw += yaw
        # 3. Convert quaternion to vectors(update _R, _P, _Q vectors)
        self._R = new_q_R.vector()
        self._P = new_q_P.vector()
        self._Q = new_q_Q.vector()
        # print("new parameters:")
        # print("R:", self._R)
        # print("P:", self._P)
        # print("Q:", self._Q)
        # 4. update parameters and calculate each wire length
        self._update_parameters()
        wires_length = [np.linalg.norm(v) for v in self._p]
        # 5. calculate torque of each motor
        tau = self._cal_tau()
        motors_tau = np.array([np.linalg.norm(v) for v in tau])
        # print("cal tau: ", tau)

        # 6. little trick, convert torques to wires' length
        max_force = motors_tau.max()
        motors_tau = (motors_tau / max_force) * (1 - ratio)
        # print("tau: ", motors_tau)
        # calculate new wire length
        new_wires_length = np.array([wires_length[i]*(1-motors_tau[i]) for i in range(len(motors_tau))])
        diff_wires_length = new_wires_length - self.previous_wires_length    # the value should be added
        # print("{} = {} - {}".format(diff_wires_length[0], self.previous_wires_length[0], new_wires_length[0]))
        # 7. update wires' length
        self.previous_wires_length = new_wires_length
        return new_wires_length, diff_wires_length


if __name__ == "__main__":
    sc = SpineControl()
    wires_length, diff_length = sc.cal_active_length(0, 0, 0, 0.95)     # original state, 0 degrees
    print("L0: ", wires_length, "D: ", diff_length)
    wires_length, diff_length = sc.cal_active_length(0, 0.5236, 0, 0.95)  # rotating 30 degrees in pitch orientation, 30 degrees
    print("L30: ", wires_length, "D: ", diff_length)
    wires_length, diff_length = sc.cal_active_length(0, -0.5236, 0, 0.95)  # rotating -30 degrees in pitch orientation, 0 degrees
    print("L0: ", wires_length, "D: ", diff_length)
    wires_length, diff_length = sc.cal_active_length(0, 0.5236, 0, 0.95)  # rotating 30 degrees in pitch orientation, 30 degrees
    print("L30: ", wires_length, "D: ", diff_length)
    wires_length, diff_length = sc.cal_active_length(0, -0.5236*2, 0, 0.95)  # rotating 30 degrees in pitch orientation, 60
    print("L-30: ", wires_length, "D: ", diff_length)
    wires_length, diff_length = sc.cal_active_length(0, 0.5236, 0, 0.95)  # rotating 30 degrees in pitch orientation, 90
    print("L90: ", wires_length, "D: ", diff_length)
    # print("D: ", diff_length*9.1)
    # print("arc: ", diff_length/0.03)
    # wires_length, diff_length = sc.cal_active_length(0, 0, 0)
    # print(wires_length)
    # print(diff_length)