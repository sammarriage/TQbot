# ----------------------------------------------------------------
# The University of York
# School of Physics, Engineering, and Technology
# Institute for Safe Autonomy
#
# File   : tqbot_shell.py
# Author : Sam Marriage
# Date   : April 2026
#
# --------------
# Allows for easier communication with tqbot. No need for raw ROS2 commands
# ----------------------------------------------------------------

# ----------------------------------------------------------------
# Imports
# ----------------------------------------------------------------
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import JointState
from rcl_interfaces.srv import SetParameters, GetParameters

from src.control.tqbot_kinematics import LegKinematics, Leg

# ------------- Bus layout -------------
BUSES = {
    'SpineA': {'ids': [0, 1]},
    'SpineB': {'ids': [0, 1]},
    'FL': {'ids': [0, 1, 2]},
    'FR': {'ids': [0, 1, 2]},
    'RL': {'ids': [0, 1, 2]},
    'RR': {'ids': [0, 1, 2]}}

# -------------  Launch configurations -------------
# NOTE: Params may need tuning more, this is just currently what I had to get something working.
#       Particularly ramp rate, currently motors move very slow for safety
LAUNCH_CONFIGS = {
    'FL': {
        'port': '/dev/ttyFL',
        'motor_ids': '[0, 1, 2]',
        'default_kp': 50.0,
        'default_kd': 3.0,
        'ramp_rate': 0.7,
        'max_position_jump': 0.5,
        'position_min': -7.0,
        'position_max': 7.0,
        'control_rate_hz': 100.0,
        'publish_rate_hz': 50.0,
    },
    'FR': {
        'port': '/dev/ttyFR',
        'motor_ids': '[0, 1, 2]',
        'default_kp': 50.0,
        'default_kd': 3.0,
        'ramp_rate': 0.7,
        'max_position_jump': 0.5,
        'position_min': -7.0,
        'position_max': 7.0,
        'control_rate_hz': 100.0,
        'publish_rate_hz': 50.0,
    },
    'RL': {
        'port': '/dev/ttyUSBA1A3',
        'motor_ids': '[0, 1, 2]',
        'default_kp': 50.0,
        'default_kd': 3.0,
        'ramp_rate': 0.7,
        'max_position_jump': 0.5,
        'position_min': -7.0,
        'position_max': 7.0,
        'control_rate_hz': 100.0,
        'publish_rate_hz': 50.0,
    },
    'RR': {
        'port': '/dev/ttyUSBA2A4',
        'motor_ids': '[0, 1, 2]',
        'default_kp': 50.0,
        'default_kd': 3.0,
        'ramp_rate': 0.7,
        'max_position_jump': 0.5,
        'position_min': -7.0,
        'position_max': 7.0,
        'control_rate_hz': 100.0,
        'publish_rate_hz': 50.0,
    }
}
"""
    Once the required USB connectors have been put into TQbot, the following should also be added above and the rear legs should use their own Rl and RR buses:
    'SpineA': {
        'port': '/dev/ttyUSBA1A3',
        'motor_ids': '[0, 1]',
        'default_kp': 70.0,
        'default_kd': 7.0,
        'ramp_rate': 0.5,
        'max_position_jump': 0.5,
        'position_min': -7.0,
        'position_max': 7.0,
        'control_rate_hz': 100.0,
        'publish_rate_hz': 50.0,
    },
    'SpineB': {
        'port': '/dev/ttyUSBA2A4',
        'motor_ids': '[0, 1]',
        'default_kp': 70.0,
        'default_kd': 7.0,
        'ramp_rate': 0.5,
        'max_position_jump': 0.5,
        'position_min': -7.0,
        'position_max': 7.0,
        'control_rate_hz': 100.0,
        'publish_rate_hz': 50.0,
    }
"""

# ------------- Kinematics -------------
'''
#------------- NOTE: THIS SECTION NEEDS ADDITIONAL WORK -------------

_LEG_KIN = {
    'FL': LegKinematics(Leg.FRONT_LEFT),
    'FR': LegKinematics(Leg.FRONT_RIGHT),
    'RL': LegKinematics(Leg.REAR_LEFT),
    'RR': LegKinematics(Leg.REAR_RIGHT),
}

# motor_id on the bus -> which IK angle (0=t1 shoulder, 1=t2 hip, 2=t3 knee)
_JOINT_TO_MOTOR = {
    'FL': {0: 0, 1: 1, 2: 2},
    'FR': {0: 0, 1: 1, 2: 2},
    'RL': {0: 0, 1: 1, 2: 2},
    'RR': {0: 0, 1: 1, 2: 2},
}

# motor sign vs IK convention
_MOTOR_SIGN = {
    'FL': {0: +1.0, 1: +1.0, 2: +1.0},
    'FR': {0: +1.0, 1: -1.0, 2: -1.0},
    'RL': {0: -1.0, 1: +1.0, 2: +1.0},
    'RR': {0: -1.0, 1: -1.0, 2: -1.0},
}

# motor offset rad (motor reading at IK zero pose). Filled by calibrate().
_MOTOR_OFFSET = {
    'FL': {0: 0.8, 1: -1.04, 2: 4.55},
    'FR': {0: 0.0, 1: 0.0, 2: 0.0},
    'RL': {0: 0.0, 1: 0.0, 2: 0.0},
    'RR': {0: 0.0, 1: 0.0, 2: 0.0},
}

# Standing posture foot targets in each leg's hip frame (metres).
STAND_TARGETS = {
    'FL': (0.0,  0.117, -0.40),
    'FR': (0.0, -0.117, -0.40),
    'RL': (0.0,  0.117, -0.40),
    'RR': (0.0, -0.117, -0.40),
}

def calibrate(bus_name):
    """
    Record current motor positions as the offsets that map to the
    IK zero pose. Hold the leg physically in the IK zero pose, then
    call this. Subsequent foot() commands will be referenced to this.

    This needs to be repeated after any power cycle (incremental encoders reset to 0).
    """
    _init()
    if bus_name not in _LEG_KIN:
        print(f"  unknown leg: {bus_name}")
        return
    _shell.pump(0.3)
    for mid in BUSES[bus_name]['ids']:
        s = _shell._latest[bus_name][mid]
        if s is None:
            print(f"  no state for {bus_name} motor {mid} — abort")
            return
        _MOTOR_OFFSET[bus_name][mid] = s['position']
    print(f"  calibrated {bus_name}: {_MOTOR_OFFSET[bus_name]}")


def calibrate_all():
    """Calibrate all four legs in sequence."""
    for leg in ['FL', 'FR', 'RL', 'RR']:
        input(f"\n  Hold {leg} in IK zero pose, then press Enter...")
        calibrate(leg)
    print("\n  All four legs calibrated. Offsets:")
    for leg in ['FL', 'FR', 'RL', 'RR']:
        print(f"    {leg}: {_MOTOR_OFFSET[leg]}")


def foot(bus_name, x, y, z):
    """
    Command a leg's foot to position (x, y, z) in its hip frame [m].
    """
    _init()
    if bus_name not in _LEG_KIN:
        print(f"  unknown leg: {bus_name}")
        return
    kin = _LEG_KIN[bus_name]
    try:
        t1, t2, t3 = kin.inverse(x, y, z)
    except Exception as e:
        print(f"  IK failed: {e}")
        return
    angles = (t1, t2, t3)
    sign = _MOTOR_SIGN[bus_name]
    offset = _MOTOR_OFFSET[bus_name]
    jmap = _JOINT_TO_MOTOR[bus_name]
    for mid in BUSES[bus_name]['ids']:
        joint_idx = jmap[mid]
        cmd = sign[mid] * angles[joint_idx] + offset[mid]
        target(bus_name, mid, cmd)
    _shell.pump(0.3)
    status_summary(bus_name)

def show_kin_tables():
    """Print the kinematics mapping / sign / offset tables."""
    print("\n=== Kinematics tables ===")
    for leg in ['FL', 'FR', 'RL', 'RR']:
        print(f"\n  {leg}:")
        print(f"    joint-to-motor:  {_JOINT_TO_MOTOR[leg]}")
        print(f"    motor signs:     {_MOTOR_SIGN[leg]}")
        print(f"    motor offsets:   {_MOTOR_OFFSET[leg]}")
    print(f"\n  STAND_TARGETS:")
    for leg, t in STAND_TARGETS.items():
        print(f"    {leg}: ({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f})")

'''
# ------------- End of Kinematics -------------

# List of processes (for debug)
_launched_processes = {}

# ------------- QoS profile matching the motor node -------------
STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


# ------------- ROS2 node -------------
class TQbotShell(Node):
    def __init__(self):
        super().__init__('tqbot_shell')

        self._pubs = {}
        self._service_clients = {}
        self._param_clients = {}
        self._latest = {}
        self._bus_status = {}
        self._subs = []

        for bus_name, info in BUSES.items():
            self._pubs[bus_name] = {}
            self._latest[bus_name] = {}
            self._bus_status[bus_name] = 'unknown'

            for mid in info['ids']:
                self._pubs[bus_name][mid] = self.create_publisher(
                    Float64,
                    f"/tqbot/{bus_name}/motor_{mid}/target",
                    10
                )
                # State topics use BEST_EFFORT
                sub = self.create_subscription(
                    JointState,
                    f"/tqbot/{bus_name}/motor_{mid}/state",
                    self._make_state_cb(bus_name, mid),
                    STATE_QOS
                )
                self._subs.append(sub)
                self._latest[bus_name][mid] = None

            status_sub = self.create_subscription(
                String,
                f"/tqbot/{bus_name}/bus_status",
                self._make_status_cb(bus_name),
                STATE_QOS
            )
            self._subs.append(status_sub)

            self._service_clients[bus_name] = {
                'enable': self.create_client(
                    SetBool, f"/tqbot/{bus_name}/enable"),
                'estop': self.create_client(
                    Trigger, f"/tqbot/{bus_name}/emergency_stop"),
                'reset_estop': self.create_client(
                    Trigger, f"/tqbot/{bus_name}/reset_estop")
            }

            # Parameter services (for set_param / get_param)
            self._param_clients[bus_name] = {
                'set': self.create_client(
                    SetParameters,
                    f"/multi_motor_node_{bus_name}/set_parameters"),
                'get': self.create_client(
                    GetParameters,
                    f"/multi_motor_node_{bus_name}/get_parameters")
            }

    def _make_state_cb(self, bus_name, mid):
        def cb(msg):
            self._latest[bus_name][mid] = {
                'position': msg.position[0] if msg.position else 0.0,
                'velocity': msg.velocity[0] if msg.velocity else 0.0,
                'effort': msg.effort[0] if msg.effort else 0.0
            }

        return cb

    def _make_status_cb(self, bus_name):
        def cb(msg):
            self._bus_status[bus_name] = msg.data

        return cb

    def pump(self, duration=0.3):
        end = time.time() + duration
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def publish_target(self, bus_name, mid, value, repeats=5):
        msg = Float64()
        msg.data = float(value)
        for _ in range(repeats):
            self._pubs[bus_name][mid].publish(msg)
            time.sleep(0.02)
        rclpy.spin_once(self, timeout_sec=0.05)

    def call_service(self, bus_name, service_name, data=None):
        client = self._service_clients[bus_name][service_name]
        if not client.wait_for_service(timeout_sec=2.0):
            return False, f"{bus_name}/{service_name}: service unavailable"

        if service_name == 'enable':
            req = SetBool.Request()
            req.data = bool(data)
        else:
            req = Trigger.Request()

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        if future.result() is None:
            return False, "call timed out"
        return future.result().success, future.result().message

    def set_parameter(self, bus_name, param_name, value):
        client = self._param_clients[bus_name]['set']
        if not client.wait_for_service(timeout_sec=2.0):
            return False, "parameter service unavailable"

        param = Parameter(param_name, Parameter.Type.DOUBLE, float(value))
        req = SetParameters.Request()
        req.parameters = [param.to_parameter_msg()]

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        if future.result() is None:
            return False, "call timed out"
        result = future.result().results[0]
        return result.successful, result.reason or "ok"

    def get_parameter_value(self, bus_name, param_name):
        client = self._param_clients[bus_name]['get']
        if not client.wait_for_service(timeout_sec=2.0):
            return None, "parameter service unavailable"

        req = GetParameters.Request()
        req.names = [param_name]

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        if future.result() is None:
            return None, "call timed out"

        val_msg = future.result().values[0]
        if val_msg.type == 3:
            return val_msg.double_value, "ok"
        elif val_msg.type == 2:
            return val_msg.integer_value, "ok"
        elif val_msg.type == 4:
            return val_msg.string_value, "ok"
        elif val_msg.type == 0:
            return None, "parameter not set on this node"
        return None, f"unsupported parameter type {val_msg.type}"


# ------------- Setup -------------
_shell = None


def _init():
    global _shell
    if _shell is None:
        rclpy.init()
        _shell = TQbotShell()


# ------------- Process management -------------
def launch(bus_name, **overrides):
    """Launch a bus node with default parameters (or overrides)."""
    if bus_name not in LAUNCH_CONFIGS:
        print(f"  unknown bus: {bus_name}")
        return

    if bus_name in _launched_processes:
        proc = _launched_processes[bus_name]
        if proc.poll() is None:
            print(f"  {bus_name} already running (PID {proc.pid})")
            return

    config = {**LAUNCH_CONFIGS[bus_name], **overrides}

    cmd = [
        'python3.10', '-m', 'src.basic.multi_motor_node',
        '--ros-args',
        '-r', f'__node:=multi_motor_node_{bus_name}',
        '-p', f'port:={config["port"]}',
        '-p', f'bus_name:={bus_name}',
        '-p', f'motor_ids:={config["motor_ids"]}',
        '-p', f'control_rate_hz:={config["control_rate_hz"]}',
        '-p', f'publish_rate_hz:={config["publish_rate_hz"]}',
        '-p', f'ramp_rate:={config["ramp_rate"]}',
        '-p', f'default_kp:={config["default_kp"]}',
        '-p', f'default_kd:={config["default_kd"]}',
        '-p', f'max_position_jump:={config["max_position_jump"]}',
        '-p', f'position_min:={config["position_min"]}',
        '-p', f'position_max:={config["position_max"]}'
    ]

    logfile_path = f'/tmp/tqbot_{bus_name}.log'
    logfile = open(logfile_path, 'w')

    proc = subprocess.Popen(
        cmd,
        stdout=logfile,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid if sys.platform != 'win32' else None,
    )
    _launched_processes[bus_name] = proc

    print(f"  launched {bus_name} (PID {proc.pid}, kp={config['default_kp']})")
    print(f"    logs: tail -f {logfile_path}")


def launch_all():
    """Launch all three configured buses."""
    for bus_name in LAUNCH_CONFIGS:
        launch(bus_name)
        time.sleep(0.3)


def kill(bus_name):
    """Kill a bus node that launched."""
    if bus_name not in _launched_processes:
        print(f"  {bus_name} was not launched from this shell")
        return

    proc = _launched_processes[bus_name]
    if proc.poll() is not None:
        print(f"  {bus_name} already exited")
        _launched_processes.pop(bus_name)
        return

    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass

    try:
        proc.wait(timeout=3)
        print(f"  {bus_name} stopped cleanly")
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        print(f"  {bus_name} force-killed")

    _launched_processes.pop(bus_name)


def kill_all():
    """Kill every bus node launched."""
    for bus_name in list(_launched_processes.keys()):
        kill(bus_name)


def processes():
    """List running processes started."""
    if not _launched_processes:
        print("  no processes launched from this shell")
        return
    for bus_name, proc in _launched_processes.items():
        alive = proc.poll() is None
        status_str = "running" if alive else f"exited (code {proc.returncode})"
        print(f"  {bus_name}: PID {proc.pid} — {status_str}")


# ------------- Parameter editing -------------
def set_param(bus_name, param_name, value):
    """
    Change a parameter on a running node
    """
    _init()
    ok, msg = _shell.set_parameter(bus_name, param_name, value)
    print(f"  set_param {bus_name}.{param_name} = {value}: "
          f"{'OK' if ok else 'FAIL'} — {msg}")
    return ok


def get_param(bus_name, param_name):
    """Read a parameter's current value from a running node."""
    _init()
    val, msg = _shell.get_parameter_value(bus_name, param_name)
    if val is None:
        print(f"  get_param {bus_name}.{param_name}: FAIL — {msg}")
    else:
        print(f"  {bus_name}.{param_name} = {val}")
    return val


# ------------- State & control API -------------
def status():
    """Print all motor states (position, velocity, effort)."""
    _init()
    _shell.pump(0.3)
    print("\n=== TQbot motor states ===")
    for bus_name in LAUNCH_CONFIGS:
        status_str = _shell._bus_status.get(bus_name, 'unknown')
        print(f"\n  [{bus_name}] — {status_str}")
        for mid in BUSES[bus_name]['ids']:
            s = _shell._latest[bus_name][mid]
            if s is None:
                print(f"    motor {mid}: (no data)")
            else:
                print(f"    motor {mid}: "
                      f"pos={s['position']:+8.4f}  "
                      f"vel={s['velocity']:+8.4f}  "
                      f"eff={s['effort']:+8.4f}")


def status_summary(*bus_names):
    """Summary for listed buses."""
    _init()
    _shell.pump(0.2)
    for bus_name in bus_names:
        bs = _shell._bus_status.get(bus_name, '?')
        parts = []
        for mid in BUSES[bus_name]['ids']:
            s = _shell._latest[bus_name].get(mid)
            if s is None:
                parts.append(f"m{mid}=(nodata)")
            else:
                parts.append(f"m{mid}={s['position']:+.3f}")
        print(f"  [{bus_name}] {bs}  " + "  ".join(parts))


def hold():
    """Print status continuously (CtrlC to stop)."""
    _init()
    print("Watching (CtrlC to stop)...")
    try:
        while True:
            status()
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopped.")


# ------------- Bus control -------------
def enable(bus_name):
    _init()
    ok, msg = _shell.call_service(bus_name, 'enable', data=True)
    print(f"  enable {bus_name}: {'OK' if ok else 'FAIL'} — {msg}")
    return ok


def disable(bus_name):
    _init()
    ok, msg = _shell.call_service(bus_name, 'enable', data=False)
    print(f"  disable {bus_name}: {'OK' if ok else 'FAIL'} — {msg}")
    return ok


def enable_all():
    for bus_name in LAUNCH_CONFIGS:
        enable(bus_name)


def disable_all():
    for bus_name in LAUNCH_CONFIGS:
        disable(bus_name)


def start(bus_name):
    launch(bus_name)
    time.sleep(0.3)
    enable(bus_name)


def stop(bus_name):
    disable(bus_name)
    time.sleep(0.3)
    kill(bus_name)


def start_all():
    for bus_name in LAUNCH_CONFIGS:
        start(bus_name)


def stop_all():
    for bus_name in LAUNCH_CONFIGS:
        stop(bus_name)


def estop(bus_name):
    _init()
    ok, msg = _shell.call_service(bus_name, 'estop')
    print(f"  ESTOP {bus_name}: {'OK' if ok else 'FAIL'} — {msg}")
    return ok


def estop_all():
    for bus_name in LAUNCH_CONFIGS:
        estop(bus_name)


def reset_estop(bus_name):
    _init()
    ok, msg = _shell.call_service(bus_name, 'reset_estop')
    print(f"  reset_estop {bus_name}: {'OK' if ok else 'FAIL'} — {msg}")
    return ok


def target(bus_name, motor_id, position_rad):
    """
    Send a motor on a bus to a target position.
    """
    _init()
    if bus_name not in BUSES:
        print(f"  unknown bus: {bus_name}")
        return
    if motor_id not in BUSES[bus_name]['ids']:
        print(f"  motor {motor_id} not on {bus_name}")
        return
    _shell.publish_target(bus_name, motor_id, position_rad)
    print(f"  {bus_name} motor {motor_id} -> {position_rad:+.4f} rad")


# ------------- Limb commanding -------------
def spine(a0_rad, a1_rad, b0_rad, b1_rad):
    """
    Send all four spine motors to target positions simultaneously.
    """
    target('SpineA', 0, a0_rad)
    target('SpineA', 1, a1_rad)
    target('SpineB', 0, b0_rad)
    target('SpineB', 1, b1_rad)
    _shell.pump(0.3)
    status_summary('SpineA', 'SpineB')


def fl(m0_rad, m1_rad, m2_rad):
    """
    Send all three front left leg motors to target positions.
    """
    target('FL', 0, m0_rad)
    target('FL', 1, m1_rad)
    target('FL', 2, m2_rad)
    _shell.pump(0.3)
    status_summary('FL')


def fr(m0_rad, m1_rad, m2_rad):
    """
    Send all three front right leg motors to target positions.
    """
    target('FR', 0, m0_rad)
    target('FR', 1, m1_rad)
    target('FR', 2, m2_rad)
    _shell.pump(0.3)
    status_summary('FR')


def rl(m0_rad, m1_rad, m2_rad):
    """
    Send all three rear left leg motors to target positions.
    """
    target('RL', 0, m0_rad)
    target('RL', 1, m1_rad)
    target('RL', 2, m2_rad)
    _shell.pump(0.3)
    status_summary('RL')


def rr(m0_rad, m1_rad, m2_rad):
    """
    Send all three front right leg motors to target positions.
    """
    target('RR', 0, m0_rad)
    target('RR', 1, m1_rad)
    target('RR', 2, m2_rad)
    _shell.pump(0.3)
    status_summary('RR')

# ------------- Help -------------
def help():
    print("""
TQbot shell — quick reference
==============================

QUICK START
  start_all()                        launch + enable every configured bus
  status()                           confirm every bus shows OK, motors have data
  spine(-2, -2, -2, -2)              send commands to all spine motors
  fl(0.0, 0.5, -1.0)                 send commands to all FL leg motors
  stand()                            execute staggered standing posture routine
  stop_all()                         disable + kill every bus
  exit()                             leave the shell

LAUNCH / KILL
  launch('FL')                       spawn one bus node, default gains
  launch('FL', default_kp=500)       spawn with a parameter override
  launch_all()                       spawn every bus in LAUNCH_CONFIGS
  kill('FL')                         SIGTERM a node we launched
  kill_all()                         kill every node we launched
  processes()                        list our launched processes + PIDs

ENABLE / DISABLE  (engage or brake PD on a running node)
  enable('FL')                       engage PD control
  disable('FL')                      brake (soft, not latched)
  enable_all() / disable_all()       every configured bus

FULL START/STOP
  start('FL')                        launch + enable in one go
  stop('FL')                         disable + kill in one go
  start_all() / stop_all()           every bus in LAUNCH_CONFIGS

EMERGENCY STOP
  estop('FL')                        latch the bus into brake
  estop_all()                        e-stop every configured bus
  reset_estop('FL')                  clear latch (bus stays disabled)

STATE INSPECTION
  status()                           full state of every motor (pos / vel / eff)
  status_summary('FL', 'FR')         compact one-line per listed bus
  hold()                             status() in a loop, CtrlC to stop

LIVE PARAMETER EDITING
  set_param('FL', 'default_kp', 400.0)
  set_param('FL', 'ramp_rate', 0.5)
  get_param('FL', 'default_kp')

  Editable parameters (all doubles):
    default_kp, default_kd           PD gains
    ramp_rate                        max command slew (rad/s, output-side)
    max_position_jump                safety: allowed gap cmd vs actual (rad)
    position_min, position_max       soft position limits (rad)

COMMAND MOTOR POSITIONS  (all values in radians, output-side)
  target('FL', 0, 1.5)
      single motor: bus, motor_id, position

  spine(a0, a1, b0, b1)
      all four spine motors at once:
        a0: SpineA motor 0    a1: SpineA motor 1
        b0: SpineB motor 0    b1: SpineB motor 1
      e.g. spine(-2, -2, -2, -2)

  fl(m0, m1, m2) / fr(...) / rl(...) / rr(...)
      all three leg motors at once (m0: shoulder, m1: hip, m2: knee)
      e.g. fl(0.0, 0.5, -1.0)

KINEMATICS & CARTESIAN CONTROL (X, Y, Z in meters inside local hip frame)
  show_kin_tables()                  print structural mappings, signs, and offsets
  calibrate('FL')                    save current motor angles as IK zero pose
  calibrate_all()                    interactive step-by-step wizard to calibrate all legs
  foot('FL', x, y, z)                send leg to 3D coordinate using Inverse Kinematics

LOGS
  Each launched bus writes to  /tmp/tqbot_<busname>.log
  Watch in a separate terminal:
      tail -f /tmp/tqbot_FL.log

EXIT
  stop_all(), exit()                 shutdown
""")


if __name__ == "__main__":
    _init()
    print("TQbot shell ready. Type help() for commands.")
