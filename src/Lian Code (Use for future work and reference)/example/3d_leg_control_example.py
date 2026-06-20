# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author    : Yunlong Lian, PhD students
# File      : 3d_leg_control_example.py
# Date      : 30-Aug-2024
# Version:  : 
# ----------------------------------------------------------------

import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

import numpy as np
from src.control.kinematics import LegIK


class SingleLegControl(LegIK):
    def __init__(self):
        super().__init__()
        pass

    def standing_foot_end(self):
        _x = np.sqrt(self.l2**2 + self.l3**2)
        _y = self.l1
        _z = 0
        return _x, _y, _z
    def origin_to_standing_angles(self):
        """
        Original position to standing position for one leg, (0,0,0) to (0,0,0)
        """
        x, y, z = self.standing_foot_end()
        a1, a2, a3 = self.inverse_kinematics(x, y, z)
        return a1, a2, a3

    def prone_to_standing_angles(self):
        x = 0.079
        y = 0.134
        z = 0.061
        a1, a2, a3 = self.inverse_kinematics(x, y, z)
        return a1, a2, a3


if __name__ == "__main__":
    test_leg = SingleLegControl()
    x, y, z = test_leg.origin_to_standing_angles()
    a, b, c = test_leg.prone_to_standing_angles()

    print(test_leg.standing_foot_end())
    print(test_leg.origin_to_standing_angles())
    print(test_leg.prone_to_standing_angles())
    print(a-x, b-y, c-z)
    # 0.35991868431468765  0.4536645614894873 - 1.2006484560459885
    # print(test_leg.inverse_kinematics(0.634999999, 0.077, 0))   # should get zero degrees for all joints


