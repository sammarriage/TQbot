# ----------------------------------------------------------------
# The University of York
# School of Physics, Engineering, and Technology
# Institute for Safe Autonomy
#
# File   : test_kinematics.py
# Author : Sam Marriage
# Date   : April 2026
# ----------------------------------------------------------------

# ----------------------------------------------------------------
# Imports
# ----------------------------------------------------------------
import math
import sys
import pytest
from tqbot_kinematics import LegKinematics, Leg, DEFAULT_L1, DEFAULT_L2, DEFAULT_L3

# ---------------------------------------------------------------------------
# Tolerance (metres)
# ---------------------------------------------------------------------------
TOLERANCE = 1e-9   # much lower than actually needed, but validates theoretical accuracy


# ---------------------------------------------------------------------------
# Pytest fixtures
# ---------------------------------------------------------------------------
@pytest.fixture(params=[
    (Leg.FRONT_LEFT,  "Front Left  (FL)"),
    (Leg.FRONT_RIGHT, "Front Right (FR)"),
    (Leg.REAR_LEFT,  "Rear Left  (RL)"),
    (Leg.REAR_RIGHT, "Rear Right (RR)"),
], ids=["FL", "FR", "RL", "RR"])
def leg_fixture(request):
    """Yields (LegKinematics, leg_label) for each configured leg."""
    leg_enum, leg_label = request.param
    return LegKinematics(leg_enum), leg_label


# ---------------------------------------------------------------------------
# Helper: run a single round trip and return residual
# ---------------------------------------------------------------------------
def round_trip(kin: LegKinematics, px: float, py: float, pz: float) -> float:
    t1, t2, t3 = kin.inverse(px, py, pz)   # IK: target to angles
    rx, ry, rz = kin.forward(t1, t2, t3)   # FK: angles to recovered position
    return math.sqrt((px - rx) ** 2 + (py - ry) ** 2 + (pz - rz) ** 2)


# ---------------------------------------------------------------------------
# Test angle sets used by test_round_trip_grid
# ---------------------------------------------------------------------------
ROUND_TRIP_ANGLES = [
    # (theta1,  theta2,  theta3)
    # ---------- theta1 = 0 ----------
    ( 0.0,  0.80, -0.50),
    ( 0.0,  1.20, -0.60),
    ( 0.0,  0.30, -0.10),
    ( 0.0,  0.70, -1.10),
    ( 0.0,  0.50, -0.90),
    ( 0.0,  0.20, -0.30),
    # ---------- theta2 = 0 ----------
    (-0.1,  0.0, -0.50),
    ( 0.2,  0.0, -0.60),
    ( 0.15, 0.0, -0.10),
    # ---------- theta3 = 0 ----------
    (-0.2,  0.70, 0.0),
    ( 0.5,  0.50, 0.0),
    ( 0.3,  0.20, 0.0),
    # ---------- mixed theta values ----------
    ( 0.30,  0.90, -0.45),
    (-0.30,  0.90, -0.45),
    ( 0.15,  1.10, -0.55),
    (-0.15,  1.10, -0.55),
    ( 0.20,  0.65, -0.30),
    (-0.20,  0.75, -0.35),
    ( 0.50,  0.80, -0.50),
]


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("t1,t2,t3", ROUND_TRIP_ANGLES)
def test_round_trip_grid(leg_fixture, t1, t2, t3):
    """IK to FK round-trip residual must be below TOLERANCE for every angle set."""
    kin, leg_label = leg_fixture
    px, py, pz = kin.forward(t1, t2, t3)
    residual = round_trip(kin, px, py, pz)
    assert residual < TOLERANCE, (
        f"[{leg_label}] Round-trip failed for angles "
        f"({math.degrees(t1):+.1f}°, {math.degrees(t2):+.1f}°, "
        f"{math.degrees(t3):+.1f}°), residual {residual:.2e} m "
        f"exceeds tolerance {TOLERANCE} m"
    )


def test_nominal_standing_posture(leg_fixture):
    """
    Verify the nominal standing posture IK FK round trip.

    Target: foot directly below the hip at 80 % maximum extension.
    """
    kin, leg_label = leg_fixture
    l1, l2, l3 = kin.l1, kin.l2, kin.l3

    stand_height = 0.80 * (l2 + l3)
    target_px = 0.0
    target_py = l1          # hip-offset distance laterally
    target_pz = -stand_height

    t1, t2, t3 = kin.inverse(target_px, target_py, target_pz)
    rx, ry, rz = kin.forward(t1, t2, t3)
    residual = math.sqrt(
        (target_px - rx) ** 2 + (target_py - ry) ** 2 + (target_pz - rz) ** 2
    )

    assert residual < TOLERANCE, (
        f"[{leg_label}] Standing posture residual {residual:.2e} m "
        f"exceeds tolerance {TOLERANCE} m"
    )


def test_fk_known_values(leg_fixture):
    """
    Check FK against manually computed expected positions.
    """
    kin, leg_label = leg_fixture
    l1, l2, l3 = kin.l1, kin.l2, kin.l3

    px, py, pz = kin.forward(0.0, 0.0, 0.0)
    assert abs(px - (l2 + l3)) < TOLERANCE, (
        f"[{leg_label}] FK(0,0,0) px={px:.6f}, expected {l2 + l3:.6f}"
    )
    assert abs(py - l1) < TOLERANCE, (
        f"[{leg_label}] FK(0,0,0) py={py:.6f}, expected {l1:.6f}"
    )
    assert abs(pz - 0.0) < TOLERANCE, (
        f"[{leg_label}] FK(0,0,0) pz={pz:.6f}, expected 0.0"
    )


def test_workspace_boundary(leg_fixture):
    """
    Targets beyond maximum reach must not raise an exception (clamping check).
    """
    kin, leg_label = leg_fixture
    l1, l2, l3 = abs(kin.l1), kin.l2, kin.l3

    beyond = (l2 + l3) * 1.5
    # Should not raise, the IK clamps c3 to [-1, 1]
    t1, t2, t3 = kin.inverse(beyond, 0.0, 0.0)
    px, py, pz = kin.forward(t1, t2, t3)
    # Just verify we got finite numbers back
    assert all(math.isfinite(v) for v in (t1, t2, t3, px, py, pz)), (
        f"[{leg_label}] Non-finite value returned for out-of-reach target"
    )


# ---------------------------------------------------------------------------
# Stand-alone runner (python test_kinematics.py)
# ---------------------------------------------------------------------------
def main():
    print("\nTQbot Leg Kinematics")
    print("---------------------------------------------------------------------------")
    print(f"Default link lengths: L1={DEFAULT_L1} m, "
          f"L2={DEFAULT_L2} m, L3={DEFAULT_L3} m")

    legs_to_test = [
        (Leg.FRONT_LEFT,  "Front Left  (FL)"),
        (Leg.FRONT_RIGHT, "Front Right (FR)"),
    ]

    all_passed = True
    for leg_enum, leg_label in legs_to_test:
        kin = LegKinematics(leg_enum)

        # FK known values
        l1, l2, l3 = kin.l1, kin.l2, kin.l3
        px, py, pz = kin.forward(0.0, 0.0, 0.0)
        fk_ok = (abs(px-(l2+l3)) < TOLERANCE and
                 abs(py-l1) < TOLERANCE and
                 abs(pz) < TOLERANCE)
        print(f"[{'PASS' if fk_ok else 'FAIL'}] FK known values, {leg_label}")

        # Round-trip grid
        rt_ok = True
        max_err = 0.0
        for t1, t2, t3 in ROUND_TRIP_ANGLES:
            p = kin.forward(t1, t2, t3)
            err = round_trip(kin, *p)
            max_err = max(max_err, err)
            if err >= TOLERANCE:
                rt_ok = False
        print(f"[{'PASS' if rt_ok else 'FAIL'}] Round-trip grid, {leg_label}  "
              f"(max err={max_err:.2e} m)")

        # Nominal standing posture
        stand_height = 0.80 * (kin.l2 + kin.l3)
        t1, t2, t3 = kin.inverse(0.0, kin.l1, -stand_height)
        rx, ry, rz = kin.forward(t1, t2, t3)
        sp_ok = math.sqrt((0-rx)**2+(kin.l1-ry)**2+(-stand_height-rz)**2) < TOLERANCE
        print(f"[{'PASS' if sp_ok else 'FAIL'}] Nominal standing posture, {leg_label}")

        # Workspace boundary
        beyond = (kin.l2 + kin.l3) * 1.5
        try:
            t1, t2, t3 = kin.inverse(beyond, 0.0, 0.0)
            fp = kin.forward(t1, t2, t3)
            wb_ok = all(math.isfinite(v) for v in fp)
        except Exception:
            wb_ok = False
        print(f"[{'PASS' if wb_ok else 'FAIL'}] Workspace boundary, {leg_label}")

        all_passed = all_passed and fk_ok and rt_ok and sp_ok and wb_ok

    print(f"\n{'='*60}")
    if all_passed:
        print("ALL TESTS PASSED")
    else:
        print("SOME TESTS FAILED ")
    print(f"{'='*60}\n")
    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())