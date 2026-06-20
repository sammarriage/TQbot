# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author    : Yunlong Lian, PhD students
# File      : my_time.py
# Date      : 23-May-2023
# Version:  : 
# ----------------------------------------------------------------
import time

time_start = time.time()
clock_start = time.perf_counter()
# pro_start = time.process_time()

# time.sleep(10)
for i in range(10000):
    last_time = time.perf_counter()
    # pro_start = time.process_time()
    a = 1+1
    # pro_end = time.process_time()
    # t = pro_end-pro_start
    # if t < 0.001:
    #     time.sleep(0.001 - t)
    # time.sleep(0.001)
    while time.perf_counter() - last_time < 0.001:
        pass

time_end = time.time()
clock_end = time.perf_counter()
# pro_end = time.process_time()

print("time() time: ", time_end - time_start)
print("perf_counter() time:", clock_end - clock_start)
# print("process_time() time:", pro_end - pro_start)