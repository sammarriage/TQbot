# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author    : Yunlong Lian, PhD students
# File      : temp_math.py
# Date      : 14-Jan-2024
# Version:  : 
# ----------------------------------------------------------------
import numpy as np
import matplotlib.pyplot as plt

step = 2*np.pi / 4

# roll = np.cos(1.0472)
# yaw = np.cos(0+3/2*np.pi)
# print(roll, ' ', yaw)

ang = 0
roll = []
yaw = []
for i in range(5):
    roll.append(np.cos(ang)*0.1)
    yaw.append(np.cos(ang+3/2*np.pi)*0.1)
    ang += step
# print(len(roll))
t = np.arange(0, 1, 1/5)
plt.plot(t, roll)
plt.plot(t, yaw)
plt.show()
print(roll)
print(yaw)