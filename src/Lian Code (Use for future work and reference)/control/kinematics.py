# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author    : Yunlong Lian, PhD students
# File      : kinematics.py
# Date      : 05-July-2023
# Version:  : 1.0.0
# ----------------------------------------------------------------
import sympy as sym


class LegIK:
    def __init__(self, l1=0.1169, l2=0.29, l3=0.346):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3

        row1 = [0, 0, 0]
        row2 = [-90 * sym.pi / 180, 0, self.l1]
        row3 = [0, self.l2, 0]
        fp = [0, self.l3, 0]
        self.dh = [row1, row2, row3, fp]
        self.theta = [sym.symbols('theta' + str(i)) for i in range(1, 4)]
        self.theta.append(0)

        self.T = self._all_tmatrix()
        self.px = 0
        self.py = 0
        self.pz = 0

    def inverse_kinematics(self, x, y, z):
        theta1 = self._theta_1(x, y)
        theta3 = self._theta_3(x, y, z)
        theta2 = self._theta_2(x, y, z, theta1, theta3)
        return float(theta1), float(theta2), float(theta3)

    def _theta_2(self, x, y, z, theta1, theta3):
        c1 = sym.cos(theta1)
        s1 = sym.sin(theta1)
        c3 = sym.cos(theta3)
        s3 = sym.sin(theta3)
        a = c1*x + s1*y
        s23_numerator = (self.l2*s3 *a - (self.l3+self.l2*c3)*z)
        c23_numerator = ((self.l3 + self.l2 * c3) * a + self.l2*s3*z)
        theta23 = sym.atan2(s23_numerator, c23_numerator)
        theta2 = theta23 - theta3
        return theta2

    def _theta_3(self, x, y, z, elbow=-1):
        # elbow = 1 : elbow up
        # elbow = -1 : elbow down
        assert elbow == 1 or elbow == -1
        m = x**2 + y**2 + z**2 - self.l3**2 - self.l2**2 - self.l1**2    # 0.2783+0.19908+0.00585225 = 0.4832
        # print("m: ", m)
        c3 = m/(self.l2*self.l3*2)
        # print("c3: ", c3)
        if c3**2 > 1:
            # print("m: ", m)
            # print("c3: ", c3)
            print("Invalid valueL: ", c3**2, ", ", c3, "should smaller than 1.0")
            raise ValueError("Warning: invalid foot position, the position extends beyond the leg workspace")
        s3 = sym.sqrt(1-c3**2) * elbow
        theta3 = sym.atan2(s3, c3)
        return theta3

    def _theta_1(self, x, y, elbow=1):
        # elbow = 1 : elbow up
        # elbow = -1 : elbow down
        assert elbow == 1 or elbow == -1
        m = x**2 + y**2 - self.l1**2
        if m > 1:
            raise ValueError("Warning: invalid foot position, the position extends beyond the leg workspace")
        k = sym.sqrt(m) * elbow
        theta1 = sym.atan2(y, x) - sym.atan2(self.l1, k)
        return theta1

    def _all_tmatrix(self):
        t = []
        for i in range(len(self.dh)):
            t.append(self._transformation_matrix(self.dh[i][0], self.dh[i][1],
                                                 self.dh[i][2], i))
        t.append(t[0] * t[1] * t[2] * t[3])
        # print(len(t))
        return t

    def _transformation_matrix(self, alpha, a, d, i):
        """
        :param alpha: alpha_(i-1) in radius
        :param a: a_(i-1) in meters
        :param d: d_i in meters
        :return: a transformation matrix from i-1 coordinates to i coordinates
        """
        tmatrix = sym.Matrix([
            [sym.cos(self.theta[i]), -sym.sin(self.theta[i]), 0, a],
            [sym.sin(self.theta[i]) * sym.cos(alpha),
             sym.cos(self.theta[i]) * sym.cos(alpha),
             -sym.sin(alpha),
             -sym.sin(alpha) * d],
            [sym.sin(self.theta[i]) * sym.sin(alpha),
             sym.cos(self.theta[i]) * sym.sin(alpha),
             sym.cos(alpha),
             sym.cos(alpha) * d],
            [0, 0, 0, 1]])
        # print(type(tmatrix))
        return tmatrix


if __name__ == "__main__":
    # TQbot DH parameters: The default posture of the leg is placed horizontally and backwards

    # another DH parameters for TQbot
    # row1 = [0, 0, 0]
    # row2 = [-90*sym.pi/180, 0, 0.0765]
    # row3 = [0, 0.3155, 0]
    # foot_end = [0, 0.37, 0]
    # dh = [row1, row2, row3, foot_end]

    # Example of (https://blog.csdn.net/weixin_45106952/article/details/118851868)
    # row1 = [0, 0, 0]
    # row2 = [90 * sym.pi / 180, 0.1, 0]
    # row3 = [0, 0.2, 0]
    # foot_end = [0, 0.3, 0]
    # dh = [row1, row2, row3, foot_end]

    test = LegIK()
    # print(test.T[1]*test.T[2]*test.T[3])
    angles = test.inverse_kinematics(0.636, 0, 0)
    print(angles)    # Theta1, 2, 3
    # print(type(angles[2]))
    # print(float(angles[1]))
    # k = LegIK()
    # print(type(k.T[3]))
    # print("Transform matrix of T[0-1]: ")
    # print(k.T[0])
    # print("Transform matrix of T[1-2]: ")
    # print(k.T[1])
    # print("Transform matrix of T[2-3]: ")
    # print(k.T[2])
    # print("Transform matrix of T[3-4]: ")
    # print(k.T[3])
    # print("Transform matrix of T[0-4]: ")
    # print(k.T[4][2, 3])

    # print("x: ", k.x4_org)
    # print("y: ", k.y4_org)
    # print("z: ", k.z4_org)

    # print(k.f_1)
    # print("Foot position in x-axis: ", k.p3_org_x)
    # print("Foot position in y-axis: ", k.p3_org_y)
    # print("Foot position in z-axis: ", k.p3_org_z)
    # print("Foot org in x-axis: ", k.x3_org)
    # print("Foot org in y-axis: ", k.y3_org)
    # print("Foot org in z-axis: ", k.z3_org)

    # print("P in x-axis: ", k.x)
    # print("P in y-axis: ", k.y)
    # print("P in z-axis: ", k.z)