# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author    : Yunlong Lian, PhD students
# File      : standing_posture_control.py
# Date      : 04-Apr-2025
# Version:  : 
# ----------------------------------------------------------------
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from src.control.kinematics import LegIK
import numpy as np
import matplotlib.pyplot as plt


class StandPosture:
    def __init__(self):
        self.lf_leg = LegIK(0.117, 0.29, 0.346)
        self.lr_leg = LegIK(0.117, 0.29, 0.346)
        self.rf_leg = LegIK(-0.117, 0.29, 0.346)
        self.rr_leg = LegIK(-0.117, 0.29, 0.346)

    def get_current_posture(self):
        return self.b_height, self.b_rpy, self.s_rpy

    def get_legs_posture(self, body_height: float, body_rpy: np.ndarray, spine_rpy: np.ndarray, width=0.4638, length=0.7632, method=0):
        self.b_height = body_height
        self.b_rpy = body_rpy
        self.s_rpy = spine_rpy
        self.width = width
        self.length = length
        self.__init_parameters()
        if method == 0:
            foot_mat = self.__cal_foot_end()
        else:
            foot_mat = self.__cal_foot_end2()
        # print("Foot End: ", foot_mat)
        foot_mat[[0, 2], :] = foot_mat[[2, 0], :]    # transform coordinate system
        foot_mat[0, :] *= -1
        # foot_mat[2, :] *= -1
        # print("Foot End: ", foot_mat)
        s, h, k = self.lf_leg.inverse_kinematics(foot_mat[0][0], foot_mat[1][0], foot_mat[2][0])
        lf_angles = np.array([s, h, k]).reshape(-1, 1)
        s, h, k = self.lr_leg.inverse_kinematics(foot_mat[0][1], foot_mat[1][1], foot_mat[2][1])
        lr_angles = np.array([s, h, k]).reshape(-1, 1)
        s, h, k = self.rf_leg.inverse_kinematics(foot_mat[0][2], foot_mat[1][2], foot_mat[2][2])
        rf_angles = np.array([s, h, k]).reshape(-1, 1)
        s, h, k = self.rr_leg.inverse_kinematics(foot_mat[0][3], foot_mat[1][3], foot_mat[2][3])
        rr_angles = np.array([s, h, k]).reshape(-1, 1)
        legs_angles = np.hstack([lf_angles, lr_angles, rf_angles, rr_angles])
        # print(lf_angles)
        # print(legs_angles)
        return legs_angles

    def __init_parameters(self):
        # all in o0 coordinate system
        self.o01 = np.array([[0.],
                             [0.],
                             [self.b_height]])  # vector o0o1
        self.o12 = np.array([[0.1825],
                             [0.],
                             [0.]])  # vector o1o2
        self.o13 = np.array([[-0.1675],  # vector o1o3
                             [0.],
                             [0.]])
        self.front_hips = np.array([[0.1991, 0.1991],  # lf, rf
                                    [0.115, -0.115],  # vector o2B1, hip position in 02 coordinate system
                                    [0., 0.]])
        self.rear_hips = np.array([[-0.2141, -0.2141],  # lr, rr, 0.2141
                                   [0.115, -0.115],
                                   [0., 0.]])
        self.foot_pos = np.array([[self.length / 2, -self.length / 2, self.length / 2, -self.length / 2],    # LF, LR, RF, RR
                                  [self.width / 2, self.width / 2, -self.width / 2, -self.width / 2],
                                  [0., 0., 0., 0.]])  # all foot position in o0 coordinate system

    def __cal_foot_end(self):
        # calculate rotation matrix
        front_spine_rot_mat = self._rot_mat(self.s_rpy)
        rear_spine_rot_mat = self._rot_mat(-self.s_rpy)
        body_rot_mat = self._rot_mat(self.b_rpy)
        fsb_rot_mat = np.dot(body_rot_mat, front_spine_rot_mat)    # front spine body rot mat
        rsb_rot_mat = np.dot(body_rot_mat, rear_spine_rot_mat)
        # print(rot_front_body)
        # print(np.dot(body_rot_mat, self.o12))
        # print(self.front_hips[:, 0:1])
        # print("body matrix: ", body_rot_mat)
        lf_hip = np.dot(body_rot_mat, self.o12) + np.dot(fsb_rot_mat, self.front_hips[:, 0:1])   # o1B1
        rf_hip = np.dot(body_rot_mat, self.o12) + np.dot(fsb_rot_mat, self.front_hips[:, 1:2])
        lr_hip = np.dot(body_rot_mat, self.o13) + np.dot(rsb_rot_mat, self.rear_hips[:, 0:1])
        rr_hip = np.dot(body_rot_mat, self.o13) + np.dot(rsb_rot_mat, self.rear_hips[:, 1:2])
        # print("lf_hip: ", lf_hip)
        # lf_hip = np.dot(body_rot_mat, self.o12) + np.dot(front_spine_rot_mat, self.front_hips[:, 0:1])  # o1B1
        # rf_hip = np.dot(body_rot_mat, self.o12) + np.dot(front_spine_rot_mat, self.front_hips[:, 1:2])
        # lr_hip = np.dot(body_rot_mat, self.o13) + np.dot(rear_spine_rot_mat, self.rear_hips[:, 0:1])
        # rr_hip = np.dot(body_rot_mat, self.o13) + np.dot(rear_spine_rot_mat, self.rear_hips[:, 1:2])

        # print("Hip: ", lf_hip, rf_hip, lr_hip, rr_hip)
        lf_foot = self.foot_pos[:, 0:1] - self.o01 - lf_hip    # o0B2 - o01 - o1B1
        lr_foot = self.foot_pos[:, 1:2] - self.o01 - lr_hip
        rf_foot = self.foot_pos[:, 2:3] - self.o01 - rf_hip
        rr_foot = self.foot_pos[:, 3:4] - self.o01 - rr_hip
        # print("lf_foot", lf_foot)
        # print(lf_foot)
        # print(rf_foot)
        # print(lr_foot)
        # print(rr_foot)
        feet_mat = np.hstack([lf_foot, lr_foot, rf_foot, rr_foot])
        # print("LF,  LR,  RF,  RR")
        # print(foot_mat)
        return feet_mat

    def __cal_foot_end2(self):
        b_mat = self._rot_mat(self.b_rpy)
        b_mat = np.hstack([b_mat, self.o01])
        b_rot_mat = np.vstack([b_mat, np.array([0, 0, 0, 1])])
        # print(b_rot_mat)
        fs_mat = self._rot_mat(self.s_rpy)
        fs_mat = np.hstack([fs_mat, self.o12])
        fs_rot_mat = np.vstack([fs_mat, np.array([0, 0, 0, 1])])
        rs_mat = self._rot_mat(-self.s_rpy)
        rs_mat = np.hstack([rs_mat, self.o13])
        rs_rot_mat = np.vstack([rs_mat, np.array([0, 0, 0, 1])])
        # print(fs_rot_mat)
        fst_mat = np.dot(b_rot_mat, fs_rot_mat)    # front spine mat
        rst_mat = np.dot(b_rot_mat, rs_rot_mat)    # rear spine mat
        # print(t_mat)
        lf_front_hip = np.vstack([self.front_hips[:, 0:1], [1]])
        rf_front_hip = np.vstack([self.front_hips[:, 1:2], [1]])
        lr_front_hip = np.vstack([self.rear_hips[:, 0:1], [1]])
        rr_front_hip = np.vstack([self.rear_hips[:, 1:2], [1]])
        # print(lf_front_hip)
        lf_hip = np.dot(fst_mat, lf_front_hip)[:-1, :]
        rf_hip = np.dot(fst_mat, rf_front_hip)[:-1, :]
        lr_hip = np.dot(rst_mat, lr_front_hip)[:-1, :]
        rr_hip = np.dot(rst_mat, rr_front_hip)[:-1, :]
        # print("New lf_hip", lf_hip)
        lf_foot = self.foot_pos[:, 0:1] - lf_hip
        lr_foot = self.foot_pos[:, 1:2] - lr_hip
        rf_foot = self.foot_pos[:, 2:3] - rf_hip
        rr_foot = self.foot_pos[:, 3:4] - rr_hip
        # print("New lf_foot", lf_foot, lr_foot)
        # print("Left  width-0.4638: ", lf_foot[1] + lr_foot[1] + 0.23, "length-0.7362: ", (lf_foot[0] + lr_foot[0] + 0.3816*2))
        # print("LR width-0.4638: ", lr_foot[1] * 2 + 0.23, "length-0.7362: ", (lr_foot[0] + 0.3816)*2)
        # print("Right width-0.4638: ", rf_foot[1] + rr_foot[1] - 0.23, "length-0.7362: ", (-rf_foot[0] - rr_foot[0] + 0.3816*2))
        # print("RR width-0.4638: ", rr_foot[1] * 2 - 0.23, "length-0.7362: ", (-rr_foot[0] + 0.3816)*2)
        # print("-----------------")
        # print("length-0.7632: ", lf_foot - lr_foot)
        feet_mat = np.hstack([lf_foot, lr_foot, rf_foot, rr_foot])
        return feet_mat

    @staticmethod
    def _rot_mat(rpy):
        rot_x = StandPosture._rot_roll(rpy[0])
        # print("rot_x: ", rot_x)
        rot_y = StandPosture._rot_pitch(rpy[1])
        rot_z = StandPosture._rot_yaw(rpy[2])
        # mat = rot_x@rot_y@rot_z
        mat = np.dot(np.dot(rot_z, rot_y), rot_x)
        # print("mat: ", mat)
        return mat

    @staticmethod
    def _rot_roll(r) -> np.ndarray:
        """
        return a rotation matrix in roll direction
        :param r: roll angle in radians
        :return: a numpy array
        """
        r = np.array([[1, 0, 0],
                      [0, np.cos(r), -np.sin(r)],
                      [0, np.sin(r), np.cos(r)]])
        return r

    @staticmethod
    def _rot_pitch(p) -> np.ndarray:
        """
        return a rotation matrix in pitch direction
        :param p: pitch angle in radians
        :return: a numpy array
        """
        r = np.array([[np.cos(p), 0, np.sin(p)],
                      [0, 1, 0],
                      [-np.sin(p), 0, np.cos(p)]])
        return r

    @staticmethod
    def _rot_yaw(y) -> np.ndarray:
        """
        return a rotation matrix in yaw direction
        :param y: yaw angle in radians
        :return: a numpy array
        """
        r = np.array([[np.cos(y), -np.sin(y), 0],
                      [np.sin(y), np.cos(y), 0],
                      [0, 0, 1]])
        return r

    def visualize_posture(self, body_height, body_rpy, spine_rpy, width=0.4638, length=0.7632):
        self.b_height = body_height
        self.b_rpy = body_rpy
        self.s_rpy = spine_rpy
        self.width = width
        self.length = length
        self.__init_parameters()

        front_spine_rot_mat = self._rot_mat(self.s_rpy)
        rear_spine_rot_mat = self._rot_mat(-self.s_rpy)
        body_rot_mat = self._rot_mat(self.b_rpy)
        fsb_rot_mat = np.dot(front_spine_rot_mat, body_rot_mat)
        rsb_rot_mat = np.dot(rear_spine_rot_mat, body_rot_mat)

        lf_hip = np.dot(body_rot_mat, self.o12) + np.dot(fsb_rot_mat, self.front_hips[:, 0:1])
        rf_hip = np.dot(body_rot_mat, self.o12) + np.dot(fsb_rot_mat, self.front_hips[:, 1:2])
        lr_hip = np.dot(body_rot_mat, self.o13) + np.dot(rsb_rot_mat, self.rear_hips[:, 0:1])
        rr_hip = np.dot(body_rot_mat, self.o13) + np.dot(rsb_rot_mat, self.rear_hips[:, 1:2])

        lf_foot = self.foot_pos[:, 0:1] - self.o01
        lr_foot = self.foot_pos[:, 1:2] - self.o01
        rf_foot = self.foot_pos[:, 2:3] - self.o01
        rr_foot = self.foot_pos[:, 3:4] - self.o01

        # print(lf_foot - lf_hip)
        # 旋转点 o12 和 o13 在全局坐标中应当先通过身体旋转转换
        global_o12 = np.dot(body_rot_mat, self.o12)
        global_o13 = np.dot(body_rot_mat, self.o13)

        # 将所有向量从列向量转为一维数组，方便绘图
        lf_hip = lf_hip.flatten()
        rf_hip = rf_hip.flatten()
        lr_hip = lr_hip.flatten()
        rr_hip = rr_hip.flatten()

        lf_foot = lf_foot.flatten()
        rf_foot = rf_foot.flatten()
        lr_foot = lr_foot.flatten()
        rr_foot = rr_foot.flatten()

        global_o12 = global_o12.flatten()
        global_o13 = global_o13.flatten()

        # 绘图
        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection='3d')

        # 绘制四个髋关节，并用线连接（连接顺序：左前 -> 右前 -> 右后 -> 左后 -> 左前）
        hips = np.array([lf_hip, rf_hip, rr_hip, lr_hip, lf_hip])
        ax.plot(hips[:, 0], hips[:, 1], hips[:, 2], 'k-', linewidth=2, label='Hip connections')
        ax.scatter(hips[:-1, 0], hips[:-1, 1], hips[:-1, 2], c='r', s=60, label='Hip joints')

        # 绘制四个足端
        feet = np.array([lf_foot, rf_foot, rr_foot, lr_foot])
        ax.scatter(feet[:, 0], feet[:, 1], feet[:, 2], c='b', s=60, label='Foot positions')

        # 绘制从每个髋关节到对应足端的连线
        for hip, foot in zip([lf_hip, rf_hip, lr_hip, rr_hip], [lf_foot, rf_foot, lr_foot, rr_foot]):
            ax.plot([hip[0], foot[0]], [hip[1], foot[1]], [hip[2], foot[2]], 'g--', linewidth=1.5)

        # 绘制旋转点 o12 和 o13
        ax.scatter(global_o12[0], global_o12[1], global_o12[2], c='m', marker='^', s=80, label='o12')
        ax.scatter(global_o13[0], global_o13[1], global_o13[2], c='c', marker='^', s=80, label='o13')

        # 添加坐标轴标签和图例
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        ax.set_title("Visualization of Hip and Foot Positions")
        plt.show()


if __name__ == "__main__":
    # b_rpy = np.array([0.5236, 0, 0])
    # b_rpy = np.array([0, 0, 0])
    # s_rpy = np.array([0, 0.5236, 0])
    # # s_rpy = np.array([0, 0, 0])
    # test_stand = StandPosture()
    # ang = test_stand.get_legs_posture(0.5, b_rpy, s_rpy)
    # print(ang)
    body_height = 0.5
    body_rpy = np.array([0.0, 0.0, 0.0])
    spine_rpy = np.array([0.0, 0.3, 0.0])
    sp = StandPosture()
    ang = sp.get_legs_posture(body_height, body_rpy, spine_rpy, method=1)
    # print(ang)
    sp.visualize_posture(body_height, body_rpy, spine_rpy)