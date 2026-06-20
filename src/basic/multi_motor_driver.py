# ----------------------------------------------------------------
# The University of York
# School of Physics, Engineering and Technology
# Institute for Safe Autonomy
#
# File   : multi_motor_driver.py
# Author : Sam Marriage
# Date   : April 2026
#
# --------------
# Defines MotorBus, a class responsible for interacting with the hardware.
# One MotorBus represents a physical serial port, and transmits data sent from
# 'motor.py' and collects the motors response.
#
# TQbot's hardware layout:
#   /dev/ttyFL       - front left leg (3 motors: shoulder, hip, knee)
#   /dev/ttyFR       - front right leg (3 motors)
#   /dev/ttyRL       - rear left  leg (3 motors) (See notes below)
#   /dev/ttyRR       - rear right  leg (3 motors) (See notes below)
#   /dev/ttyUSBA1A3  - spine bus A (2 motors) (NOTE: This has been temporarily repurposed as LR due to lack of USB convertors)
#   /dev/ttyUSBA2A4  - spine bus B (2 motors) (NOTE: This has been temporarily repurposed as RR due to lack of USB convertors)
# ----------------------------------------------------------------


# ----------------------------------------------------------------
# Imports
# ----------------------------------------------------------------

# ------------- Generic imports -------------
import os
import sys
import threading
from typing import Optional

# ------------- SDK library -------------
# Path to SDK library
actuator_sdk_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../unitree_actuator_sdk/lib'))
sys.path.append(actuator_sdk_path)

# ----- Import relevant from SDK: -----
# - SerialPort, transmits cmd and receives data
from unitree_actuator_sdk import SerialPort

# ------------- Import Motor class -------------
from src.basic.motor import Motor


# ----------------------------------------------------------------
# Motor Bus
# ----------------------------------------------------------------
class MotorBus:
    """
    One RS485 port shared between up to N motors.
    """

    # ------------------------------------------------------------------
    # Constructor
    # ------------------------------------------------------------------
    def __init__(self, port_path: str, name: Optional[str] = None) -> None:
        self.port_path = port_path          # Device path
        if name is not None:                # User friendly name
            self.name = name                    #If not set, use port name
        else:
            self.name = port_path

        self.serial = SerialPort(port_path) # Open the serial port
        self.motors: list[Motor] = []       # List of motors registered on this bus

        self._lock = threading.Lock()       # Thread to prevent communication mismatch
        self._emergency_stopped = False     # Emergency stop, initially False

    # ------------------------------------------------------------------
    # Motor registration
    # ------------------------------------------------------------------
    def add_motor(self, motor: Motor) -> None:
        """
        Register a Motor on this bus, rejects duplicate entries
        No hardware interaction
        """
        if any(m.motor_id == motor.motor_id for m in self.motors):
            raise ValueError(f"[{self.name}] motor id {motor.motor_id} already on bus")
        self.motors.append(motor)

    def find_motor(self, motor_id: int) -> Motor:
        """Return the Motor with the given id, or raise KeyError."""
        for m in self.motors:
            if m.motor_id == motor_id:
                return m
        raise KeyError(f"[{self.name}] no motor with id {motor_id} on this bus")

    # ------------------------------------------------------------------
    # Emergency stop
    # ------------------------------------------------------------------
    def emergency_stop(self) -> None:
        """
        Emergency stop

        All subsequent send_all_commands() calls will transmit brake
        commands regardless of what the motors' cmd buffers contain.
        Call reset_emergency_stop() to clear.
        """
        with self._lock:
            self._emergency_stopped = True
            for m in self.motors:
                m.prepare_stop_command()
        self.send_all_commands(force=True)

    def reset_emergency_stop(self) -> None:
        """Clear the emergency stop.  Motors remain in brake until commanded."""
        with self._lock:
            self._emergency_stopped = False

    def is_emergency_stopped(self) -> bool:
        return self._emergency_stopped

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def send_all_commands(self, force: bool = False) -> dict[int, bool]:
        """
        Transmit the current cmd for each motor, receive the response,
        and update the motor's state attributes.
        """
        results: dict[int, bool] = {}

        # Lock port for thread
        with self._lock:
            # If emergency stop, overwrite cmd's with brake
            if self._emergency_stopped and not force:
                for m in self.motors:
                    m.prepare_stop_command()

            # Transit motor cmd's sequentially
            for m in self.motors:
                try:
                    self.serial.sendRecv(m.cmd, m.data)
                    m.update_states()
                    results[m.motor_id] = True
                # Record any failure and proceed to next motor
                except Exception as exc:
                    print(f"[{self.name}] motor {m.motor_id} sendRecv failed: {exc}")
                    results[m.motor_id] = False

        return results

    def update_all_states(self) -> dict[int, bool]:
        return self.send_all_commands()

    # ------------------------------------------------------------------
    # Brake
    # ------------------------------------------------------------------
    def brake_all(self) -> None:
        """Soft stop: command every motor to brake mode and transmit."""
        for m in self.motors:
            m.prepare_stop_command()
        self.send_all_commands()

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------
    def any_motor_overheating(self, threshold_c: int = 40) -> bool:
        """True if any motor is above the temperature threshold."""
        return any(m.temperature > threshold_c for m in self.motors)

    def any_motor_errored(self) -> bool:
        """True if any motor is reporting a non-zero error code."""
        return any(m.error_code != 0 for m in self.motors)

    def state_summary(self) -> str:
        """One line per motor, human readable status message."""
        header = f"=== Bus {self.name} ({len(self.motors)} motors) ==="
        if self._emergency_stopped:
            header += "  [E-STOP ACTIVE]"
        lines = [header] + [m.state_str() for m in self.motors]
        return "\n".join(lines)

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------
    def __enter__(self) -> "MotorBus":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Brake all motors on exit, even if an exception was raised."""
        try:
            self.brake_all()
        except Exception as exc:
            print(f"[{self.name}] failed to brake on exit: {exc}")

    def __repr__(self) -> str:
        ids = [m.motor_id for m in self.motors]
        return (f"MotorBus(name={self.name!r}, port={self.port_path!r}, motors={ids}, e_stop={self._emergency_stopped})")
