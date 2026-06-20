# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author    : Yunlong Lian, PhD students
# File      : 2d_leg_control_example.py
# Date      : 04-Jun-2023
# Version:  : 
# ----------------------------------------------------------------
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
import numpy as np
import time


# lc = LegControl()
# t1, t2 = lc.get_angles(685.5, 0)
# a1 =t1 * 180/np.pi
# a2 =t2 * 180/np.pi
# print("In rad", t1, " ", t2)
# print("In deg", a1, " ", a2)
def get_angle_step(pos, angle_fre, ctrl_fre):
    current_pos = 0
    if pos < current_pos:
        g = -1  # rotation direction
    else:
        g = 1
    if ctrl_fre > 1000:
        ctrl_fre = 1000
        print("control frequency is set up to 1000 as it is larger than maximum value 1000")
    angle_step = g * angle_fre * (np.pi / 180) * 9.1 / ctrl_fre
    return angle_step
# parameters:
hip = 10
knee = -50

ang_fre = 50
ct_fre = 1000

hip_step = get_angle_step(hip, ang_fre, ct_fre)
knee_step = get_angle_step(knee, ang_fre, ct_fre)
current_hip = 0
current_knee = 0
achieve_hip = False
achieve_knee = False
time_interval = 1/ct_fre
# move two joints at the same time
hip_t = 0
hip_time = time.perf_counter()
knee_t = 0
knee_time = time.perf_counter()
start_time = time.perf_counter()
while not achieve_hip or not achieve_knee:    # waiting for completing rotate two joints
    last_time = time.perf_counter()
    if abs(current_hip - hip) < 0.00873 and not achieve_hip:
        achieve_hip = True
        hip_step = 0
        print("hip aim achieved: ", time.perf_counter() - hip_time)
        # print("hip aim achieved: ", self.hip_motor.position/9.1 * (180 / np.pi))
        # print("current knee angle: ", current_knee)
    if abs(current_knee - knee) < 0.00873 and not achieve_knee:
        achieve_knee = True
        knee_step = 0
        print("knee aim achieved", time.perf_counter() - knee_time)
        # print("knee aim achieved: ", knee / 9.1 * (180 / np.pi)/2)
        # print("current hip angle: ", current_hip / 9.1 * 180 / np.pi)

    # print("sent hip joint")
    # print("sent knee joint")
    # print("sent")
    if not achieve_hip:
        hip_t += 1
    current_hip += hip_step
    if not achieve_knee:
        knee_t += 1
    current_knee += knee_step
    while time.perf_counter() - last_time < time_interval:
        pass
print("overall time: ", time.perf_counter() - start_time)
print("times: ", hip_t, " ", knee_t)
print("Hip/knee joint completed")