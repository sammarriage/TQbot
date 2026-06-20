# ----------------------------------------------------------------
# The University of York
# School of Physics, Engineering and Technology
# Institute for Safe Autonomy
#
# File   : multi_motor_node.py
# Author : Sam Marriage
# Date   : April 2026
#
# --------------
#
# ROS2 node wrapping a single MotorBus.  One instance per physical
# RS485 bus on TQbot.  Publishes motor state and accepts position
# commands over ROS2 topics.
#
# Relationship to other files:
#   Motor               single motor state + command formatting + safety
#   MotorBus            owns SerialPort, dispatches to N Motors
#   MultiMotorNode      this file, ROS2 wrapper around MotorBus
#
# ----------------------------------------------------------------

# ----------------------------------------------------------------
# Imports
# ----------------------------------------------------------------

# ------------- Generic imports -------------
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import JointState

# ------------- Import Motor, MotorSafetyConfig, and MotorBus classes -------------
from src.basic.motor import Motor, MotorSafetyConfig
from src.basic.multi_motor_driver import MotorBus


# ----------------------------------------------------------------
# QoS profiles
# ----------------------------------------------------------------
# State
STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Command
COMMAND_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# ----------------------------------------------------------------
# Multi motor node
# ----------------------------------------------------------------
class MultiMotorNode(Node):
    """
    ROS2 node wrapping a single MotorBus
    """

    def __init__(self) -> None:
        super().__init__("multi_motor_node")

        # ---------- parameters ----------
        self.declare_parameter("port", "/dev/ttyLF")
        self.declare_parameter("bus_name", "LF")
        self.declare_parameter("motor_ids", [0, 1, 2])
        self.declare_parameter("control_rate_hz", 300.0)
        self.declare_parameter("publish_rate_hz", 100.0)

        # Output-side safety limits (applied to all motors on this bus).
        # Override per-motor in future by declaring params.
        self.declare_parameter("max_velocity", 10.0)          # rad/s
        self.declare_parameter("max_torque", 20.0)            # Nm
        self.declare_parameter("position_min", -3.14)         # rad
        self.declare_parameter("position_max", 3.14)          # rad
        self.declare_parameter("max_position_jump", 0.05)     # rad per update
        self.declare_parameter("ramp_rate", 0.5)              # rad/s max slew

        # Default PD gains (output-side, tuned conservatively low for safety)
        self.declare_parameter("default_kp", 8.0)
        self.declare_parameter("default_kd", 2.0)

        # ---------- resolved values ----------
        self.port = self.get_parameter("port").value
        self.bus_name = self.get_parameter("bus_name").value
        self.motor_ids = list(self.get_parameter("motor_ids").value)
        self.control_rate = float(self.get_parameter("control_rate_hz").value)
        self.publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.ramp_rate = float(self.get_parameter("ramp_rate").value)
        self.default_kp = float(self.get_parameter("default_kp").value)
        self.default_kd = float(self.get_parameter("default_kd").value)

        safety = MotorSafetyConfig(
            max_velocity=float(self.get_parameter("max_velocity").value),
            max_torque=float(self.get_parameter("max_torque").value),
            position_min=float(self.get_parameter("position_min").value),
            position_max=float(self.get_parameter("position_max").value),
            max_position_jump=float(
                self.get_parameter("max_position_jump").value
            ),
        )

        self.get_logger().info(
            f"[{self.bus_name}] starting on {self.port} "
            f"with motors {self.motor_ids}, ctrl={self.control_rate} Hz, "
            f"pub={self.publish_rate} Hz"
        )

        # ---------- set up the bus and motors ----------
        self.bus = MotorBus(self.port, name=self.bus_name)
        for mid in self.motor_ids:
            self.bus.add_motor(Motor(motor_id=mid, safety=safety))

        # ---------- node runtime state ----------
        # Target output-side positions, per motor id.  None until enabled.
        self._targets: dict[int, float] = {mid: 0.0 for mid in self.motor_ids}
        # Currently commanded (ramped) positions, per motor id.
        self._commanded: dict[int, float] = {
            mid: 0.0 for mid in self.motor_ids
        }
        self._enabled = False
        self._state_lock = threading.Lock()

        # ---------- ROS2 interfaces ----------
        cb_group = ReentrantCallbackGroup()

        # Per-motor publishers and subscribers
        self._state_pubs: dict[int, rclpy.publisher.Publisher] = {}
        self._target_subs: dict[int, rclpy.subscription.Subscription] = {}
        for mid in self.motor_ids:
            topic_base = f"/tqbot/{self.bus_name}/motor_{mid}"
            self._state_pubs[mid] = self.create_publisher(
                JointState, f"{topic_base}/state", STATE_QOS,
            )
            self._target_subs[mid] = self.create_subscription(
                Float64,
                f"{topic_base}/target",
                self._make_target_callback(mid),
                COMMAND_QOS,
                callback_group=cb_group,
            )

        # Bus-wide diagnostics publisher
        self._status_pub = self.create_publisher(
            String, f"/tqbot/{self.bus_name}/bus_status", STATE_QOS,
        )

        # Services
        self._enable_srv = self.create_service(
            SetBool, f"/tqbot/{self.bus_name}/enable",
            self._on_enable, callback_group=cb_group,
        )
        self._estop_srv = self.create_service(
            Trigger, f"/tqbot/{self.bus_name}/emergency_stop",
            self._on_emergency_stop, callback_group=cb_group,
        )
        self._reset_srv = self.create_service(
            Trigger, f"/tqbot/{self.bus_name}/reset_estop",
            self._on_reset_estop, callback_group=cb_group,
        )

        # ---------- timers ----------
        self._control_timer = self.create_timer(
            1.0 / self.control_rate,
            self._control_tick,
            callback_group=cb_group,
        )
        self._publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self._publish_tick,
            callback_group=cb_group,
        )

        self.get_logger().info(
            f"[{self.bus_name}] ready; disabled until /enable is called."
        )

    # ----------------------------------------------------------------
    # Callbacks
    # ----------------------------------------------------------------
    def _make_target_callback(self, motor_id: int):
        """Closure that captures motor_id for its subscription."""
        def cb(msg: Float64) -> None:
            with self._state_lock:
                self._targets[motor_id] = float(msg.data)
        return cb

    def _on_enable(
        self, request: SetBool.Request, response: SetBool.Response,
    ) -> SetBool.Response:
        """
        On enable, set the target for each motor to its current position
        so nothing moves.
        """
        with self._state_lock:
            if request.data:
                if self.bus.is_emergency_stopped():
                    response.success = False
                    response.message = (
                        "cannot enable while e-stop is active; "
                        "call /reset_estop first"
                    )
                    self.get_logger().warn(response.message)
                    return response

                # Read current positions (send brake cmd first so read
                # has no mechanical effect)
                self.bus.brake_all()
                for m in self.bus.motors:
                    current_out = m.rotor_to_output_angle(m.position)
                    self._targets[m.motor_id] = current_out
                    self._commanded[m.motor_id] = current_out

                self._enabled = True
                response.success = True
                response.message = f"[{self.bus_name}] enabled"
                self.get_logger().info(response.message)
            else:
                self._enabled = False
                self.bus.brake_all()
                response.success = True
                response.message = f"[{self.bus_name}] disabled"
                self.get_logger().info(response.message)
        return response

    def _on_emergency_stop(
        self, request: Trigger.Request, response: Trigger.Response,
    ) -> Trigger.Response:
        with self._state_lock:
            self._enabled = False
            self.bus.emergency_stop()
        response.success = True
        response.message = f"[{self.bus_name}] E-STOP latched"
        self.get_logger().error(response.message)
        return response

    def _on_reset_estop(
        self, request: Trigger.Request, response: Trigger.Response,
    ) -> Trigger.Response:
        self.bus.reset_emergency_stop()
        response.success = True
        response.message = (
            f"[{self.bus_name}] e-stop cleared; bus remains disabled "
            f", call /enable to resume"
        )
        self.get_logger().info(response.message)
        return response

    # ----------------------------------------------------------------
    # Control tick, runs at control_rate_hz
    # ----------------------------------------------------------------
    def _control_tick(self) -> None:
        """
        Prepare and send a command for every motor, once per tick.
        """
        if self.bus.is_emergency_stopped():
            # bus will force brake anyway, but call send to cycle the loop
            self.bus.send_all_commands()
            return

        if not self._enabled:
            self.bus.brake_all()
            return

        # ---------- diagnostic tick counter ----------
        if not hasattr(self, '_tick_count'):
            self._tick_count = 0
        self._tick_count += 1
        log_this_tick = (self._tick_count % 100 == 0)  # once per second at 100 Hz

        # Max change per control tick, derived from ramp rate
        max_step = self.ramp_rate / self.control_rate

        with self._state_lock:
            # ---------- diagnostics (print targets vs commanded) ----------
            if log_this_tick:
                debug_info = ", ".join(
                    f"m{mid}: tgt={self._targets[mid]:+.4f} cmd={self._commanded[mid]:+.4f}"
                    for mid in self.motor_ids
                )
                self.get_logger().info(f"[{self.bus_name}] TARGETS {debug_info}")

            for m in self.bus.motors:
                target = self._targets[m.motor_id]
                commanded = self._commanded[m.motor_id]

                # Ramp toward target
                delta = target - commanded
                if abs(delta) > max_step:
                    delta = max_step if delta > 0 else -max_step
                commanded += delta
                self._commanded[m.motor_id] = commanded

                # Prepare position command
                ok = m.prepare_position_command(
                    q_out=commanded,
                    kp_out=self.default_kp,
                    kd_out=self.default_kd,
                )
                if not ok:
                    self.get_logger().warn(
                        f"[{self.bus_name}] motor {m.motor_id} command "
                        f"failed safety check; braking bus"
                    )
                    self._enabled = False
                    self.bus.brake_all()
                    return

                #---------- diagnostic: print what's being sent to SDK ----------
                if log_this_tick:
                    self.get_logger().info(
                        f"[{self.bus_name}] m{m.motor_id} SDK "
                        f"q_cmd={m.cmd.q:+.4f} q_act={m.position:+.4f} "
                        f"kp={m.cmd.kp:.4f} kd={m.cmd.kd:.4f} "
                        f"tau_cmd={m.cmd.tau:+.3f} tau_act={m.torque:+.3f}"
                    )

        # Transmit all prepared commands
        results = self.bus.send_all_commands()
        for mid, ok in results.items():
            if not ok:
                self.get_logger().warn(
                    f"[{self.bus_name}] motor {mid} sendRecv failure"
                )

    # ----------------------------------------------------------------
    # Publish tick
    # ----------------------------------------------------------------
    def _publish_tick(self) -> None:
        """
        Publish each motor's state and a bus status string.
        """
        now = self.get_clock().now().to_msg()

        for m in self.bus.motors:
            msg = JointState()
            msg.header.stamp = now
            msg.name = [f"motor_{m.motor_id}"]
            msg.position = [m.rotor_to_output_angle(m.position)]
            msg.velocity = [m.rotor_to_output_velocity(m.velocity)]
            msg.effort = [m.torque]   # Nm (already output-side)
            self._state_pubs[m.motor_id].publish(msg)

        # Bus status
        status = String()
        flags = []
        if self.bus.is_emergency_stopped():
            flags.append("ESTOP")
        if not self._enabled:
            flags.append("DISABLED")
        if self.bus.any_motor_overheating():
            flags.append("HOT")
        if self.bus.any_motor_errored():
            flags.append("MOTOR_ERR")
        status.data = (
            f"{self.bus_name}: " + (",".join(flags) if flags else "OK")
        )
        self._status_pub.publish(status)

    # ----------------------------------------------------------------
    # Shutdown
    # ----------------------------------------------------------------
    def destroy_node(self) -> bool:
        """Brake all motors before destroying the node."""
        try:
            self.bus.brake_all()
        except Exception as exc:
            self.get_logger().error(f"brake on shutdown failed: {exc}")
        return super().destroy_node()


# ----------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = MultiMotorNode()

    # Multi-threaded executor so timers, services, and subscriptions
    # can run in parallel.  ReentrantCallbackGroup on callbacks lets
    # them overlap; the MotorBus lock still prevents bus corruption.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()