# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author    : Yunlong Lian, PhD students
# File      : spine_control_example.py
# Date      : 08-Aug-2023
# Version:  : 1.0.0
# ----------------------------------------------------------------
import numpy as np

# unit in m

# Real definition
# R = np.array([-0.05, 0, 0.068])
# P = np.array([-0.05, 0.135, -0.068])
# Q = np.array([-0.05, -0.135, -0.068])

# r = np.array([[0.05, -0.135, 0.068],
#              [0.05, 0.135, 0.068],
#              [0.05, -0.135, -0.068],
#              [0.05, 0.135, -0.068]])

# Chrono definition
R = np.array([-0.05, 0, 0.06])
P = np.array([-0.05, -0.135, -0.06])
Q = np.array([-0.05, 0.135, -0.06])

r = np.array([[0.05, 0.135, 0.06],
             [0.05, -0.135, 0.06],
             [0.05, 0.135, -0.06],
             [0.05, -0.135, -0.06]])

p = np.array([R-r[0],
             R-r[1],
             Q-r[2],
             P-r[3]])

wire_length = [np.linalg.norm(v) for v in p]

print("little p: ", p)    # Structural matrix
print("wire length: ", wire_length)
# print("P shape", P.shape)
# print("r shape", r.shape)
# print("p shape", p.shape)
P = P.reshape(3, 1)
R = R.reshape(3, 1)
Q = Q.reshape(3, 1)
# print("P shape", P.shape)
# print("r shape", r.shape)
# print("p shape", p.shape)
w = np.cross(r, p)
w_t = w.T
# i_matrix = np.identity(4)


# print(w_t)
# for i in range(len(r)):
#     print(np.cross(r[i], p[i]))
# print(np.cross(r[0], p[0]))

# print(P)
# print(R)
# print(Q)
# print(r)
# print(p)
w_pinv = np.linalg.pinv(w_t)
# print(w_pinv)
# print(i_matrix - w_pinv.dot(w_t))

n_matrix = w[:-1].T
w4 = w[-1].reshape(3, 1)
# tau4 = np.array([[0.122, 0.233, 0.344]])    # 1x3
tau4 = np.array([[5, 5, 5]])    # 1x3
n_inv = -np.linalg.inv(n_matrix)
a = n_inv.dot(w4)
# the following may not be needed
tau_n = a.dot(tau4)
all_tau = np.append(tau_n, tau4, axis=0)

print("structural matrix . T", w)
print("structural matrix_3", n_matrix)
print("structural matrix_3_inverse", n_inv)
print("The 4th structural vector", w4)
print("n_inv*w 1-3", a)
print("all torques", all_tau)
# print(np.matmul(n_inv, w4))
# print(all_tau)

# print(np.matmul(a, tau4))
# print(tau_n)
# print(w_t.dot(all_tau))

# Tianyuan's method
ratio = 0.95
forces = np.append(a, [[10]], 0)
max_force = forces.max()
forces = (forces / max_force) * (1 - ratio)
wire_length = [wire_length[i]*(1-forces[i]) for i in range(len(forces))]
print(wire_length)

# Paper's method
forces = np.array([np.linalg.norm(v) for v in all_tau])
print("paper's torques", forces)
forces.max()
forces = (forces / max_force) * (1 - ratio)
wire_length = [wire_length[i]*(1-forces[i]) for i in range(len(forces))]
print(wire_length)



