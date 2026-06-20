# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author    : Yunlong Lian, PhD students
# File      : quaternion.py
# Date      : 05-July-2023
# Version:  : 1.0.0
# ----------------------------------------------------------------
import numpy as np
import math


class Quaternion:
    def __init__(self, w, x, y, z):
        """
        create a rotation quaternion
        :param w: cos(theta/2)
        :param x: x*sin(theta/2)
        :param y: y*sin(theta/2)
        :param z: z*sin(theta/2)
        """
        # self.w = sym.cos(rotation_angle/2)
        # self.x = x * sym.sin(rotation_angle/2)
        # self.y = y * sym.sin(rotation_angle/2)
        # self.z = z * sym.sin(rotation_angle/2)

        self.w = w
        self.x = x
        self.y = y
        self.z = z
    @staticmethod
    def euler_to_Q(roll, pitch, yaw):
        """
        roll-pitch-yaw: c-b-a
        :param roll: angle in radian
        :param pitch: angle in radian
        :param yaw: angle in radian
        :return: Quaternion
        """
        roll = roll/2
        pitch = pitch/2
        yaw = yaw/2
        w = np.cos(roll)*np.cos(pitch)*np.cos(yaw) + np.sin(roll)*np.sin(pitch)*np.sin(yaw)
        x = np.sin(roll)*np.cos(pitch)*np.cos(yaw) - np.cos(roll)*np.sin(pitch)*np.sin(yaw)
        y = np.cos(roll)*np.sin(pitch)*np.cos(yaw) + np.sin(roll)*np.cos(pitch)*np.sin(yaw)
        z = np.cos(roll)*np.cos(pitch)*np.sin(yaw) - np.sin(roll)*np.sin(pitch)*np.cos(yaw)

        return Quaternion(w, x, y, z)

    def __getitem__(self, item):
        return [self.w, self.x, self.y, self.z][item]

    def __mul__(self, q):
        w = self.w * q.w - self.x * q.x - self.y * q.y - self.z * q.z
        x = self.w * q.x + self.x * q.w + self.y * q.z - self.z * q.y
        y = self.w * q.y - self.x * q.z + self.y * q.w + self.z * q.x
        z = self.w * q.z + self.x * q.y - self.y * q.x + self.z * q.w
        return Quaternion(w, x, y, z)

    # def __str__(self):
    #     return "Quaternion[{}, {}, {}, {}]".format(self.w, self.x, self.y, self.z)

    def __repr__(self):
        return "w={}, x={}, y={}, z={}".format(self.w, self.x, self.y, self.z)

    def conjugate(self):
        """
        Get the conjugate of the quaternion
        :return: Quaternion(w, -x, -y, -z)
        """
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def vector(self):
        """
        Convert quaternion to vector
        :return: numpy.array([x, y, z])
        """
        return np.array([self.x, self.y, self.z])

if __name__ == '__main__':
    import sympy as sym

    ang = 0.345
    # q = sym.Quaternion(0.707, 0, 0, 0.707)
    # p = sym.Quaternion(1, 0, 0, 0)
    # print(q.conjugate())
    # print(q.mul(p))

    # q1 = Quaternion(0.707, 0, 0, 0.707)
    # p1 = Quaternion(1, 0, 0, 0)
    # print(q1*p1)

    rotation_q = Quaternion.euler_to_Q(0.349, 0.349, 0.349)    # roll, pitch, yaw
    print(rotation_q)
    # no rotation
    # print(p1 * q1)
    # print(p1 * q1 * p1.conjugate())