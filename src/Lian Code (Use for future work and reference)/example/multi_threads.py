# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author    : Yunlong Lian, PhD students
# File      : multi_threads.py
# Date      : 28-May-2023
# Version:  : 
# ----------------------------------------------------------------
import threading, time
STOP_F = False
def thread_1(par):
    for i in range(par):
        last_time = time.perf_counter()
        time.sleep(0.5)
        # print(threading.current_thread().name, " ", par, " ", time.perf_counter()-last_time)
        print("In {} thread, {} threads are running".format(threading.current_thread().name, threading.active_count()))

def thread_2(par):
    for i in range(5):
        if STOP_F:
            break
        last_time = time.perf_counter()
        time.sleep(1)
        # print(threading.current_thread().name, " ", par, " ", time.perf_counter()-last_time)
        print("In {} thread, {} threads are running: {}".format(threading.current_thread().name, threading.active_count(), i))
    print("Thread 2 is over: ", STOP_F)
# thread1: threading.Thread = None
# print("thread 1 is alive? ", thread1.is_alive())
# print("thread 1 type: ", type(thread1))

thread1 = threading.Thread(target=thread_1, args=[1])
thread2 = threading.Thread(target=thread_2, args=[20])
thread1.start()


for i in range(5):
    time.sleep(0.1)
    print("In {} thread, {} threads are running".format(threading.current_thread().name, threading.active_count()))

# thread1.daemon = True
thread2.daemon = True
thread2.start()
time.sleep(1)
STOP_F = True
print("thread 1 is alive? ", thread1.is_alive())
print("thread 1 type: ", type(thread1))