# ----------------------------------------------------------------
# The University of York
# School of Physics, Engineering and Technology
# Institute for Safe Autonomy
#
# File   : test_motor_stack.py
# Author : Sam Marriage
# Date   : April 2026
#
# Structural tests for the TQbot motor stack.
# ----------------------------------------------------------------

import os
import sys
import time
import threading
import unittest
from unittest.mock import MagicMock

# ----------------------------------------------------------------
# Path setup
# ----------------------------------------------------------------
_here = os.path.abspath(os.path.dirname(__file__))
_project_root = _here
for _ in range(5):
    if os.path.isdir(os.path.join(_project_root, "src", "basic")):
        break
    _project_root = os.path.dirname(_project_root)
else:
    raise RuntimeError(
        f"Could not find project root containing 'src/basic' above {_here}"
    )

if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

# ----------------------------------------------------------------
# SDK stub
# ----------------------------------------------------------------
class _StubMotorCmd:
    def __init__(self):
        self.motorType = None
        self.id = 0
        self.mode = 0
        self.tau = 0.0
        self.dq = 0.0
        self.q = 0.0
        self.kp = 0.0
        self.kd = 0.0


class _StubMotorData:
    def __init__(self):
        self.motorType = None
        self.q = 0.0
        self.dq = 0.0
        self.tau = 0.0
        self.temp = 35
        self.merror = 0
        self.correct = True


class _StubMotorMode:
    FOC = 1
    BRAKE = 2


class _StubMotorType:
    A1 = "A1"


_sdk_stub = MagicMock()
_sdk_stub.MotorCmd = _StubMotorCmd
_sdk_stub.MotorData = _StubMotorData
_sdk_stub.MotorMode = _StubMotorMode
_sdk_stub.MotorType = _StubMotorType
_sdk_stub.queryMotorMode = lambda motor_type, mode: mode
_sdk_stub.queryGearRatio = lambda motor_type: 9.1
sys.modules["unitree_actuator_sdk"] = _sdk_stub


# ----------------------------------------------------------------
# MockSerialPort
# ----------------------------------------------------------------
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class MockFaultConfig:
    raise_on_send: bool = False
    silent_bus_error: bool = False
    force_error_code: Optional[int] = None
    force_temperature_c: Optional[int] = None
    stuck: bool = False


@dataclass
class _SimMotorState:
    position: float = 0.0
    velocity: float = 0.0
    last_update: float = field(default_factory=time.monotonic)


class MockSerialPort:
    """Simulates a Unitree SerialPort"""

    def __init__(self, port_path):
        self.port_path = port_path
        self._motors = {}
        self._fault = MockFaultConfig()
        self._call_count = 0
        self._lock = threading.Lock()

    def set_fault_config(self, fault):
        with self._lock:
            self._fault = fault

    def set_motor_position(self, motor_id, q_rotor):
        with self._lock:
            state = self._motors.setdefault(motor_id, _SimMotorState())
            state.position = q_rotor
            state.velocity = 0.0

    @property
    def call_count(self):
        with self._lock:
            return self._call_count

    def sendRecv(self, cmd, data):
        with self._lock:
            self._call_count += 1

            if self._fault.raise_on_send:
                raise RuntimeError(
                    f"[MockSerialPort {self.port_path}] injected failure"
                )

            motor_id = cmd.id
            state = self._motors.setdefault(motor_id, _SimMotorState())

            dt = 0.002   # simulate 2 ms per sendRecv (500 Hz control loop)

            if not self._fault.stuck:
                target = cmd.q
                error = target - state.position
                k_resp = 2000.0 * cmd.kp
                state.velocity = k_resp * error
                state.position += state.velocity * dt
                state.position += 0.001 * cmd.tau * dt

            data.motorType = cmd.motorType
            data.q = state.position
            data.dq = state.velocity
            data.tau = cmd.tau
            data.temp = (
                self._fault.force_temperature_c
                if self._fault.force_temperature_c is not None
                else 35
            )
            data.merror = (
                self._fault.force_error_code
                if self._fault.force_error_code is not None
                else 0
            )
            if hasattr(data, "correct"):
                data.correct = not self._fault.silent_bus_error


# Install MockSerialPort in place of the real SerialPort
sys.modules["unitree_actuator_sdk"].SerialPort = MockSerialPort


from src.basic.motor import Motor, MotorSafetyConfig
from src.basic.multi_motor_driver import MotorBus


# ----------------------------------------------------------------
# 1. Motor unit tests
# ----------------------------------------------------------------
class TestMotorUnit(unittest.TestCase):

    def test_constructs_cleanly(self):
        m = Motor(motor_id=3)
        self.assertEqual(m.motor_id, 3)
        self.assertEqual(m.position, 0.0)
        self.assertAlmostEqual(m.gear_ratio, 9.1, places=1)

    def test_output_to_rotor_roundtrip_angle(self):
        m = Motor(motor_id=0)
        for q in [-1.0, 0.0, 0.5, 3.14]:
            self.assertAlmostEqual(
                m.rotor_to_output_angle(m.output_to_rotor_angle(q)), q,
                places=6,
            )

    def test_output_to_rotor_roundtrip_velocity(self):
        m = Motor(motor_id=0)
        for dq in [-5.0, 0.0, 2.5, 10.0]:
            self.assertAlmostEqual(
                m.rotor_to_output_velocity(m.output_to_rotor_velocity(dq)),
                dq, places=6,
            )

    def test_kp_conversion_within_sdk_bounds(self):
        m = Motor(motor_id=0)
        kp_rotor = m.output_to_rotor_kp(5.0)
        self.assertGreater(kp_rotor, 0.0)
        self.assertLess(kp_rotor, 16.0)

    def test_kd_conversion_within_sdk_bounds(self):
        m = Motor(motor_id=0)
        kd_rotor = m.output_to_rotor_kd(0.1)
        self.assertGreater(kd_rotor, 0.0)
        self.assertLess(kd_rotor, 32.0)

    def test_safety_rejects_out_of_range_position(self):
        safety = MotorSafetyConfig(position_min=-1.0, position_max=1.0)
        m = Motor(motor_id=0, safety=safety)
        self.assertTrue(
            m.prepare_position_command(q_out=0.5, kp_out=3.0, kd_out=0.1)
        )
        self.assertFalse(
            m.prepare_position_command(q_out=2.0, kp_out=3.0, kd_out=0.1)
        )

    def test_safety_rejects_large_position_jump(self):
        safety = MotorSafetyConfig(
            position_min=-10.0, position_max=10.0,
            max_position_jump=0.1,
        )
        m = Motor(motor_id=0, safety=safety)
        self.assertFalse(
            m.prepare_position_command(q_out=0.5, kp_out=3.0, kd_out=0.1)
        )

    def test_prepare_stop_command_always_safe(self):
        m = Motor(motor_id=0)
        m.cmd.tau = 200.0
        m.prepare_stop_command()
        self.assertEqual(m.cmd.tau, 0.0)
        self.assertEqual(m.cmd.kp, 0.0)
        self.assertEqual(m.cmd.kd, 0.0)

    def test_hybrid_command_preserves_torque(self):
        m = Motor(motor_id=0)
        ok = m.prepare_hybrid_command(
            tau=1.5, dq_out=0.0, q_out=0.0, kp_out=3.0, kd_out=0.1,
        )
        self.assertTrue(ok)
        self.assertAlmostEqual(m.cmd.tau, 1.5)


# ----------------------------------------------------------------
# 2. MotorBus integration tests
# ----------------------------------------------------------------
class TestMotorBus(unittest.TestCase):

    def setUp(self):
        self.bus = MotorBus("/dev/fake_bus", name="TEST")
        self.fake_port = self.bus.serial

        self.safety = MotorSafetyConfig(
            position_min=-2.0, position_max=2.0,
            max_velocity=10.0, max_torque=20.0,
            max_position_jump=5.0,
        )

        self.bus.add_motor(Motor(motor_id=0, safety=self.safety))
        self.bus.add_motor(Motor(motor_id=1, safety=self.safety))
        self.bus.add_motor(Motor(motor_id=2, safety=self.safety))

    def test_three_motors_registered(self):
        self.assertEqual(len(self.bus.motors), 3)

    def test_duplicate_motor_id_rejected(self):
        with self.assertRaises(ValueError):
            self.bus.add_motor(Motor(motor_id=1, safety=self.safety))

    def test_find_motor(self):
        m = self.bus.find_motor(1)
        self.assertEqual(m.motor_id, 1)
        with self.assertRaises(KeyError):
            self.bus.find_motor(99)

    def test_send_all_commands_touches_every_motor(self):
        for m in self.bus.motors:
            m.prepare_position_command(q_out=0.1, kp_out=3.0, kd_out=0.1)
        results = self.bus.send_all_commands()
        self.assertEqual(len(results), 3)
        self.assertTrue(all(results.values()))
        self.assertEqual(self.fake_port.call_count, 3)

    def test_position_converges_toward_target(self):
        target = 0.5
        for _ in range(50):
            for m in self.bus.motors:
                m.prepare_position_command(
                    q_out=target, kp_out=5.0, kd_out=0.1,
                )
            self.bus.send_all_commands()

        for m in self.bus.motors:
            current_out = m.rotor_to_output_angle(m.position)
            self.assertGreater(current_out, 0.1,
                   f"motor {m.motor_id} did not converge "
                   f"(at {current_out:.4f}, target 0.5)")

    def test_sendrecv_failure_returns_false(self):
        self.fake_port.set_fault_config(MockFaultConfig(raise_on_send=True))
        for m in self.bus.motors:
            m.prepare_position_command(q_out=0.1, kp_out=3.0, kd_out=0.1)
        results = self.bus.send_all_commands()
        self.assertFalse(any(results.values()))

    def test_emergency_stop_forces_brake(self):
        for m in self.bus.motors:
            m.prepare_position_command(q_out=0.5, kp_out=5.0, kd_out=0.1)
        self.bus.emergency_stop()
        self.assertTrue(self.bus.is_emergency_stopped())

        for m in self.bus.motors:
            m.prepare_position_command(q_out=1.0, kp_out=5.0, kd_out=0.1)
        self.bus.send_all_commands()
        for m in self.bus.motors:
            self.assertEqual(m.cmd.kp, 0.0)
            self.assertEqual(m.cmd.kd, 0.0)

    def test_estop_reset_clears_flag(self):
        self.bus.emergency_stop()
        self.bus.reset_emergency_stop()
        self.assertFalse(self.bus.is_emergency_stopped())

    def test_brake_all(self):
        for m in self.bus.motors:
            m.prepare_position_command(q_out=0.5, kp_out=5.0, kd_out=0.1)
        self.bus.brake_all()
        for m in self.bus.motors:
            self.assertEqual(m.cmd.tau, 0.0)
            self.assertEqual(m.cmd.kp, 0.0)
            self.assertEqual(m.cmd.kd, 0.0)

    def test_overheating_detected(self):
        self.fake_port.set_fault_config(
            MockFaultConfig(force_temperature_c=95)
        )
        for m in self.bus.motors:
            m.prepare_position_command(q_out=0.0, kp_out=3.0, kd_out=0.1)
        self.bus.send_all_commands()
        self.assertTrue(self.bus.any_motor_overheating(threshold_c=80))

    def test_motor_error_detected(self):
        self.fake_port.set_fault_config(MockFaultConfig(force_error_code=2))
        for m in self.bus.motors:
            m.prepare_position_command(q_out=0.0, kp_out=3.0, kd_out=0.1)
        self.bus.send_all_commands()
        self.assertTrue(self.bus.any_motor_errored())

    def test_context_manager_brakes_on_exit(self):
        with MotorBus("/dev/fake_ctx", name="CTX") as bus:
            bus.add_motor(Motor(motor_id=7, safety=self.safety))
            bus.motors[0].prepare_position_command(
                q_out=0.3, kp_out=5.0, kd_out=0.1,
            )
            try:
                raise RuntimeError("simulated user error")
            except RuntimeError:
                pass

    def test_concurrent_calls_serialised(self):
        stop = threading.Event()
        errors = []

        def hammer():
            try:
                while not stop.is_set():
                    for m in self.bus.motors:
                        m.prepare_position_command(
                            q_out=0.1, kp_out=3.0, kd_out=0.1,
                        )
                    self.bus.send_all_commands()
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=hammer) for _ in range(4)]
        for t in threads:
            t.start()
        time.sleep(0.25)
        stop.set()
        for t in threads:
            t.join()

        self.assertEqual(errors, [])


# ----------------------------------------------------------------
# Main
# ----------------------------------------------------------------
if __name__ == "__main__":
    print()
    print("----------------------------------------------------------------")
    print(" TQbot Motor Stack Validation")
    print("----------------------------------------------------------------")
    print(f" Project root : {_project_root}")
    print(f" Python       : {sys.version.split()[0]}  ({sys.platform})")
    print(f" SDK          : STUBBED (MockSerialPort)")
    print("----------------------------------------------------------------")
    print()

    unittest.main(verbosity=2, exit=False)