# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author    : Yunlong Lian, PhD students
# File      : multi_class.py
# Date      : 24-May-2023
# Version:  : 
# ----------------------------------------------------------------
class base1:
    def __init__(self):
        super().__init__()
        print("base1")

class base2:
    def __init__(self):
        super().__init__()
        print("base2")

class base3:
    def __init__(self):
        super().__init__()
        print("base3")

class mid(base1, base2, base3):
    def __init__(self):
        super().__init__()
        print('mid')

class top(mid):
    def __init__(self):
        super().__init__()
        print('top')

if __name__ == "__main__":
    t = top()
    print('-----')
    # m = mid()