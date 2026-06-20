# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author    : Yunlong Lian, PhD students
# File      : leg_control.py
# Date      : 04-Jun-2023
# Version:  : 1.0.0
# ----------------------------------------------------------------

# 3d leg control model with hip and knee joints.
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

import math as m
from src.control.kinematics import LegIK

class LegControl(LegIK):
    def __init__(self):
        super().__init__()
        self.thigh_length = 315.5
        self.calf_length = 370
        pass

    def _cal_theta_1(self, x_pos, y_pos, theta2):
        """
        calculate the angle of theta 1, which is the angle between thigh and horizontal axis.
        Counterclockwise rotation is positive direction
        :param x_pos: foot end position on the x-axis
        :param y_pos: foot end position on the y-axis
        :param theta2: the angle between calf and extension line of thigh
        :return: theta 1 angle
        """
        theta1 = m.atan2(y_pos, x_pos) - m.atan2(self.calf_length*m.sin(theta2),
                                                 self.thigh_length+self.calf_length*m.cos(theta2))
        return theta1

    def _cal_theta_2(self, x_pos, y_pos):    # calculate the knee joint angle
        """
        This function limit arcos in range of (-pi, 0)
        :param x_pos:
        :param y_pos:
        :return: theta2 in radians
        """
        cos_theta_2 = (x_pos**2 + y_pos**2 - self.thigh_length**2 - self.calf_length**2)\
                      / (2*self.thigh_length*self.calf_length)
        # print(cos_theta_2)
        return -m.acos(cos_theta_2)

    def get_angles(self, x_pos, y_pos):
        """
        Given foot end position in (x_pos,y_pos) to calculate the theta1 and theta2 angles
        position in mm
        :param x_pos: x position of the foot (mm)
        :param y_pos: y position of the foot (mm)
        :return: theta1, theta2 (in radians)
        """
        theta2 = self._cal_theta_2(x_pos, y_pos)
        theta1 = self._cal_theta_1(x_pos, y_pos, theta2)
        return theta1, theta2

    def get_all_angles(self, foot_x, foot_y, foot_z):
        return self.inverse_kinematics(foot_x, foot_y, foot_z)

if __name__ == "__main__":
    t1 = LegControl()
    angles = t1.get_angles(400, 0)
    print(angles)

    t2 = LegIK()
    # print(test.T[1]*test.T[2]*test.T[3])
    angles = t2.inverse_kinematics(0.4, 0.0765, 0)
    print(angles)  # X Y Z