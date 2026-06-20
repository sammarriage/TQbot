# ----------------------------------------------------------------
# The University of York
# School of Physics, Engineering, and Technology
# Institute for Safe Autonomy
#
# File   : motor.py
# Author : Sam Marriage
# Date   : April 2026
#
# --------------
# Defines one class representing a single motor, acting as its 'brain':
#   - what was the motor last told to do? (cmd)
#   - what did the motor last report? (data)
#   - define the motors safety limits
#   - convert between output (user friendly) and rotor (SDK friendly) values
#
# This file does not directly talk to hardware. For this, see 'multi_motor_driver.py'
# ----------------------------------------------------------------


# ----------------------------------------------------------------
# Imports
# ----------------------------------------------------------------

# ------------- Generic imports -------------
import os
import sys
from dataclasses import dataclass
from typing import Optional

# ------------- SDK library -------------
# Path to SDK library
actuator_sdk_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../unitree_actuator_sdk/lib'))
sys.path.append(actuator_sdk_path)

# ----- Import relevant from SDK: -----
# - MotorCmd, tells the motor what to do
# - MotorData, motor reply with current state
# - MotorMode, active (FOC), or stop (BRAKE)
# - MotorType, motor model (A1)
# - queryMotorMode, converts enum to an integer the SDK can read
# - queryGearRatio, returns  gear ratio for motor type (9.1 for A1)
from unitree_actuator_sdk import (MotorCmd, MotorData, MotorMode, MotorType,queryMotorMode, queryGearRatio)


# ----------------------------------------------------------------
# A1 motor datasheet constants
# ----------------------------------------------------------------
# These convert output side (user friendly) gains into raw rotor side (SDK friendly) values.
# The gear ratio is 9.1:1
# The extra factors come from the A1 motor datasheet.
KP_OUTPUT_TO_ROTOR = 26.07   # kp divisor
KD_OUTPUT_TO_ROTOR = 100.0   # kd multiplier

# SDK bounds
SDK_MAX_TORQUE = 128.0          # Nm at rotor
SDK_MAX_VELOCITY = 256.0        # rad/s at rotor
SDK_MAX_POSITION = 823549.0     # rad at rotor
SDK_KP_BOUNDS = (0.0, 16.0)     # kp min/max rotor side
SDK_KD_BOUNDS = (0.0, 32.0)     # kd min/max rotor side

# A1 datasheet maxima
A1_MAX_TORQUE_OUTPUT = 33.5      # Nm
A1_MAX_VELOCITY_OUTPUT = 21.0    # rad/s


# ----------------------------------------------------------------
# Motor safety settings
# ----------------------------------------------------------------
@dataclass
class MotorSafetyConfig:
    """
    Soft limits (output side) which user can set individually per motor.
    Used in addition to the SDK-level hard limits.
    Defaults match A1 datasheet
    """
    max_torque: float = A1_MAX_TORQUE_OUTPUT          # Nm
    max_velocity: float = A1_MAX_VELOCITY_OUTPUT      # rad/s
    position_min: float = -float('inf')               # rad
    position_max: float = float('inf')                # rad
    max_position_jump: float = float('inf')           # rad, rejects commands
                                                        # that would cause a
                                                        # dangerous sudden move


# ----------------------------------------------------------------
# Motor
# ----------------------------------------------------------------
class Motor:
    """
    Representation of a single motor on TQbot.

    A Motor holds:
      - its motor ID on its bus
      - a MotorCmd buffer (the next command to be sent)
      - a MotorData buffer (the most recent state received)
      - current-state attributes (position, velocity, torque)
      - safety configuration

    This class does not send any data, that is handled in multi_motor_driver.py
    """

    # ------------------------------------------------------------------
    # Constructor
    # ------------------------------------------------------------------
    def __init__(self, motor_id: int, safety: Optional[MotorSafetyConfig] = None, default_mode: MotorMode = MotorMode.FOC) -> None:
        # ------------- Motor config -------------
        self.motor_id = motor_id                                        # Motor number
        if safety is not None:                                          # User safety settings recognition
            self.safety = safety
        else:
            self.safety = MotorSafetyConfig()
        self.gear_ratio = queryGearRatio(MotorType.A1)                  # Gear ratio from the SDK (9.1 for A1)
        self._run_mode = queryMotorMode(MotorType.A1, default_mode)     # Run mode
        self._stop_mode = queryMotorMode(MotorType.A1, MotorMode.BRAKE) # Brake mode

        # ------------- Communication buffers -------------
        self.cmd = MotorCmd()       # Tell the motor what to do
        self.stop_cmd = MotorCmd()  # Emergency stop
        self.data = MotorData()     # Current motor state (sent from the motor)
        self._init_buffers()        # Initialise buffers with defaults

        # ------------- Current motor states -------------
        self.position: float = 0.0      # rad, rotor side
        self.velocity: float = 0.0      # rad/s, rotor side
        self.torque: float = 0.0        # Nm
        self.temperature: int = 0       # degC
        self.error_code: int = 0        # SDK error flag

    # ------------------------------------------------------------------
    # Initialise buffers
    # ------------------------------------------------------------------
    def _init_buffers(self) -> None:
        # ------------- Active command -------------
        self.cmd.motorType = MotorType.A1   # A1 motor
        self.cmd.id = self.motor_id         # Motor number
        self.cmd.mode = self._run_mode      # Run mode (Active)
        self.cmd.tau = 0.0                  # feed-forward torque (Nm)
        self.cmd.dq = 0.0                   # desired velocity (rad/s, rotor side)
        self.cmd.q = 0.0                    # desired position (rad, rotor-side)
        self.cmd.kp = 0.0                   # position stiffness (SDK side)
        self.cmd.kd = 0.0                   # velocity damping (SDK side)

        # ------------- Stop command -------------
        self.stop_cmd.motorType = MotorType.A1  # A1 motor
        self.stop_cmd.id = self.motor_id        # Motor number
        self.stop_cmd.mode = self._stop_mode    # Stop mode (Brake)

        # ------------- Data buffer -------------
        self.data.motorType = MotorType.A1  # A1 motor

    # ------------------------------------------------------------------
    # Unit conversion
    # ------------------------------------------------------------------
    def output_to_rotor_angle(self, q_out: float) -> float:
        """Convert output side angle (rad) to rotor side angle (rad)."""
        return q_out * self.gear_ratio

    def rotor_to_output_angle(self, q_rot: float) -> float:
        """Convert rotor side angle (rad) to output side angle (rad)."""
        return q_rot / self.gear_ratio

    def output_to_rotor_velocity(self, dq_out: float) -> float:
        """Convert output side velocity (rad/s) to rotor side (rad/s)."""
        return dq_out * self.gear_ratio

    def rotor_to_output_velocity(self, dq_rot: float) -> float:
        """Convert rotor side velocity (rad/s) to output side (rad/s)."""
        return dq_rot / self.gear_ratio

    def output_to_rotor_kp(self, kp_out: float) -> float:
        """Convert output side stiffness to SDK rotor side kp."""
        return (kp_out / (self.gear_ratio ** 2)) / KP_OUTPUT_TO_ROTOR

    def output_to_rotor_kd(self, kd_out: float) -> float:
        """Convert output side damping to SDK rotor side kd."""
        return (kd_out / (self.gear_ratio ** 2)) * KD_OUTPUT_TO_ROTOR

    # ------------------------------------------------------------------
    # State update, called by MotorBus after sendRecv
    # ------------------------------------------------------------------
    def update_states(self) -> None:
        """Copy the received data into the state attributes."""
        self.position = self.data.q         # rad, rotor side
        self.velocity = self.data.dq        # rad/s, rotor side
        self.torque = self.data.tau         # Nm
        self.temperature = self.data.temp   # degC
        self.error_code = self.data.merror  # SDK error flag

    # ------------------------------------------------------------------
    # Safety checks
    # ------------------------------------------------------------------
    def basic_safety_check(self) -> bool:
        """
        Verify current command values are within SDK absolute bounds.
        Returns True if safe, False otherwise.
        """
        # Torque
        if abs(self.cmd.tau) > SDK_MAX_TORQUE:
            print(f"[Motor {self.motor_id}] tau {self.cmd.tau} > {SDK_MAX_TORQUE}")
            return False
        # Velocity
        if abs(self.cmd.dq) > SDK_MAX_VELOCITY:
            print(f"[Motor {self.motor_id}] dq {self.cmd.dq} > {SDK_MAX_VELOCITY}")
            return False
        # Position
        if abs(self.cmd.q) > SDK_MAX_POSITION:
            print(f"[Motor {self.motor_id}] q {self.cmd.q} > {SDK_MAX_POSITION}")
            return False
        # Stiffness
        if not (SDK_KP_BOUNDS[0] <= self.cmd.kp <= SDK_KP_BOUNDS[1]):
            print(f"[Motor {self.motor_id}] kp {self.cmd.kp} outside {SDK_KP_BOUNDS}")
            return False
        # Damping
        if not (SDK_KD_BOUNDS[0] <= self.cmd.kd <= SDK_KD_BOUNDS[1]):
            print(f"[Motor {self.motor_id}] kd {self.cmd.kd} outside {SDK_KD_BOUNDS}")
            return False
        return True

    def position_rotation_check(self, target_rotor_angle: float) -> bool:
        """
        Reject commands that would cause a large jump.
        """
        jump = abs(target_rotor_angle - self.position)
        max_jump_rotor = self.output_to_rotor_angle(self.safety.max_position_jump) # Convert output side to rotor side
        if jump > max_jump_rotor:
            print(f"[Motor {self.motor_id}] position jump {jump:.3f} > allowed {max_jump_rotor:.3f} (rotor)")
            return False
        return True

    def output_position_in_range(self, q_out: float) -> bool:
        """Check a position is within this motor's soft limits."""
        if not (self.safety.position_min <= q_out <= self.safety.position_max):
            print(f"[Motor {self.motor_id}] q_out {q_out:.3f} outside [{self.safety.position_min}, {self.safety.position_max}]")
            return False
        return True

    # ------------------------------------------------------------------
    # Command preparation for transmission
    # ------------------------------------------------------------------
    def _set_cmd(self, tau: float, dq: float, q: float, kp: float, kd: float, mode: Optional[MotorMode] = None) -> bool:
        """
        Set all five control parameters and run basic safety.
        All values are rotor side.
        """
        if mode is not None:
            self.cmd.mode = queryMotorMode(MotorType.A1, mode)
        else:
            self.cmd.mode = self._run_mode
        self.cmd.tau = tau
        self.cmd.dq = dq
        self.cmd.q = q
        self.cmd.kp = kp
        self.cmd.kd = kd
        return self.basic_safety_check()

    def prepare_position_command(self, q_out: float, kp_out: float, kd_out: float) -> bool:
        """
        Populate cmd for position control.

        Returns False if the command would violate safety
        If False, do not send via the bus.
        """
        if not self.output_position_in_range(q_out):
            return False

        q_rotor = self.output_to_rotor_angle(q_out)

        if not self.position_rotation_check(q_rotor):
            return False

        kp_rotor = self.output_to_rotor_kp(kp_out)
        kd_rotor = self.output_to_rotor_kd(kd_out)
        return self._set_cmd(0.0, 0.0, q_rotor, kp_rotor, kd_rotor)

    def prepare_torque_command(self, tau: float) -> bool:
        """Populate cmd for feedforward torque."""
        return self._set_cmd(tau, 0.0, 0.0, 0.0, 0.0)

    def prepare_impedance_command(self, kp_out: float, kd_out: float) -> bool:
        """Populate cmd to act as a spring/damper about 0"""
        kp_rotor = self.output_to_rotor_kp(kp_out)
        kd_rotor = self.output_to_rotor_kd(kd_out)
        return self._set_cmd(0.0, 0.0, 0.0, kp_rotor, kd_rotor)

    def prepare_speed_command(self, dq_out: float, kd_out: float) -> bool:
        """Populate cmd for velocity."""
        dq_rotor = self.output_to_rotor_velocity(dq_out)
        kd_rotor = self.output_to_rotor_kd(kd_out)
        return self._set_cmd(0.0, dq_rotor, 0.0, 0.0, kd_rotor)

    def prepare_hybrid_command(self, tau: float, dq_out: float, q_out: float, kp_out: float, kd_out: float) -> bool:
        """Generic population command"""
        if not self.output_position_in_range(q_out):
            return False
        q_rotor = self.output_to_rotor_angle(q_out)
        dq_rotor = self.output_to_rotor_velocity(dq_out)
        kp_rotor = self.output_to_rotor_kp(kp_out)
        kd_rotor = self.output_to_rotor_kd(kd_out)
        return self._set_cmd(tau, dq_rotor, q_rotor, kp_rotor, kd_rotor)

    def prepare_stop_command(self) -> None:
        """Populate cmd for an emergency stop."""
        self.cmd.mode = self._stop_mode
        self.cmd.tau = 0.0
        self.cmd.dq = 0.0
        self.cmd.q = 0.0
        self.cmd.kp = 0.0
        self.cmd.kd = 0.0

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------
    def state_str(self) -> str:
        """Human readable state summary."""
        return (
            f"Motor {self.motor_id}: "
            f"q={self.rotor_to_output_angle(self.position):+.4f} rad_out "
            f"dq={self.rotor_to_output_velocity(self.velocity):+.4f} rad/s_out "
            f"tau={self.torque:+.3f} Nm "
            f"T={self.temperature} C "
            f"err={self.error_code}"
        )