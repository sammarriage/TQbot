# ----------------------------------------------------------------
# The University of York
# School of Physics, Engineering, and Technology
# Institute for Safe Autonomy
#
# File   : tqbot_kinematics.py
# Author : Sam Marriage
# Date   : April 2026
#
# --------------
# Joint mapping
# --------------
#  theta1 = shoulder (abduction/adduction)
#  theta2 = hip      (flexion/extension)
#  theta3 = knee     (flexion/extension)
# ----------------------------------------------------------------

# ----------------------------------------------------------------
# Imports
# ----------------------------------------------------------------
import math
import numpy as np
from enum import Enum


# ----------------------------------------------------------------
# Default link lengths  (metres)
# ----------------------------------------------------------------
DEFAULT_L1 = 0.117   # hip offset
DEFAULT_L2 = 0.290   # thigh length
DEFAULT_L3 = 0.346   # calf length


class Leg(Enum):
    """Leg identifiers used to set the correct l1 sign."""
    FRONT_LEFT  = "FL"
    FRONT_RIGHT = "FR"
    REAR_LEFT   = "RL"
    REAR_RIGHT  = "RR"


# Negate right side hip offset
_RIGHT_LEGS = {Leg.FRONT_RIGHT, Leg.REAR_RIGHT}


# ----------------------------------------------------------------
# Helper
# ----------------------------------------------------------------
def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


# ----------------------------------------------------------------
# Main kinematics class
# ----------------------------------------------------------------
class LegKinematics:
    """
    Forward and Inverse Kinematics for a single TQbot leg.
    """

    def __init__(self, leg: Leg, l1: float = DEFAULT_L1, l2: float = DEFAULT_L2, l3: float = DEFAULT_L3,):
        self.leg = leg
        self.l2 = l2
        self.l3 = l3
        # Right side legs negate the hip offset (mirrored)
        if leg in _RIGHT_LEGS:
            self.l1 = -l1
        else:
            self.l1 = l1

    # ------------------------------------------------------------------
    # Forward Kinematics  (Eq. 3.13, p.84)
    # ------------------------------------------------------------------
    def forward(self, theta1: float, theta2: float, theta3: float) -> tuple[float, float, float]:
        """
        Compute foot position in the hip frame from joint angles.

        Parameters
        ----------
        theta1 = shoulder angle [rad]
        theta2 = hip angle [rad]
        theta3 = knee angle [rad]

        Returns
        -------
        (px, py, pz) : tuple[float, float, float]
        Foot tip position in the shoulder-joint frame [m].
        """
        c1, c2, c3, c23 = math.cos(theta1), math.cos(theta2), math.cos(theta3), math.cos(theta2 + theta3)
        s1, s2, s3, s23 = math.sin(theta1), math.sin(theta2), math.sin(theta3), math.sin(theta2 + theta3)

        l1, l2, l3 = self.l1, self.l2, self.l3

        # Translation column of Eq. 3.13, p.84
        px = (l3 * c1 * c23) - (l1 * s1) + (l2 * c1 * c2)
        py = (l3 * s1 * c23) + (l1 * c1) + (l2 * s1 * c2)
        pz = -((l3 * s23) + (l2 * s2))

        return px, py, pz

    # ------------------------------------------------------------------
    # Inverse Kinematics  (Eq. 3.23–3.36, pp. 86–88)
    # ------------------------------------------------------------------
    def inverse(self, px: float, py: float, pz: float) -> tuple[float, float, float]:
        """
        Compute joint angles from a desired foot position.

        Parameters
        ----------
        px, py, pz : float
        Desired foot tip position in the shoulder-joint frame [m].

        Returns
        -------
        (theta1, theta2, theta3) : tuple[float, float, float]
        Joint angles [rad] for shoulder, hip, and knee.

        Raises
        ------
        ValueError
            If the target is outside the reachable workspace (|c3| > 1).
        """
        l1, l2, l3 = self.l1, self.l2, self.l3

        # ------------- Step 1: theta1 -------------
        rho_sq = px ** 2 + py ** 2
        inner = rho_sq - l1 ** 2
        if inner < 0:
            inner = 0.0
        theta1 = math.atan2(py, px) - math.atan2(l1, math.sqrt(inner))

        # -------------  Step 2: theta3 -------------
        c3_num = px ** 2 + py ** 2 + pz ** 2 - l3 ** 2 - l2 ** 2 - l1 ** 2
        c3_den = 2.0 * l2 * l3
        c3 = c3_num / c3_den
        c3 = _clamp(c3, -1.0, 1.0)
        s3 = math.sqrt(1.0 - c3 ** 2)
        theta3 = math.atan2(s3, c3)

        # -------------  Step 3: theta2 -------------
        c1, s1 = math.cos(theta1), math.sin(theta1)

        a = c1 * px + s1 * py         # Eq. 3.34
        D = a ** 2 + pz ** 2          # denominator (Eq. 3.32–3.33)

        if D < 1e-12:
            theta2 = 0.0
        else:
            lc3 = l3 + l2 * c3
            s23 = (l2 * s3 * a - lc3 * pz) / D
            c23 = (lc3 * a + l2 * s3 * pz) / D
            theta23 = math.atan2(s23, c23)
            theta2 = theta23 - theta3

        return theta1, theta2, theta3

    # ------------------------------------------------------------------
    # Full transformation
    # ------------------------------------------------------------------
    def transform_matrix(
        self, theta1: float, theta2: float, theta3: float
    ) -> np.ndarray:
        """
        Return the full transformation matrix (Eq. 3.13).
        """
        c1, s1 = math.cos(theta1), math.sin(theta1)
        c2, s2 = math.cos(theta2), math.sin(theta2)
        c3, s3 = math.cos(theta3), math.sin(theta3)
        c23 = math.cos(theta2 + theta3)
        s23 = math.sin(theta2 + theta3)

        l1, l2, l3 = self.l1, self.l2, self.l3

        T = np.array([
            [c1*c23,  -c1*s23,  -s1,   l3*c1*c23 - l1*s1 + l2*c1*c2],
            [s1*c23,  -s1*s23,   c1,   l3*s1*c23 + l1*c1 + l2*s1*c2],
            [-s23,    -c23,       0,   -(l3*s23 + l2*s2)             ],
            [0,        0,         0,    1                             ],
        ])
        return T