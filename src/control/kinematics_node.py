# ----------------------------------------------------------------
# The University of York
# School of Physics, Engineering, and Technology
# Institute for Safe Autonomy
#
# File   : kinematics_node.py
# Author : Sam Marriage
# Date   : April 2026
#
# --------------
# Topic Layout
# --------------
# Subscribed:
# /tqbot/foot_target/fl    geometry_msgs/msg/Point
# /tqbot/foot_target/fr    geometry_msgs/msg/Point
# /tqbot/foot_target/rl    geometry_msgs/msg/Point
# /tqbot/foot_target/rr    geometry_msgs/msg/Point
#
# Published:
# /tqbot/joint_angles/fl   tqbot_msgs/msg/LegJoints
# /tqbot/joint_angles/fr   tqbot_msgs/msg/LegJoints
# /tqbot/joint_angles/rl   tqbot_msgs/msg/LegJoints
# /tqbot/joint_angles/rr   tqbot_msgs/msg/LegJoints
#
# Published (FK, for diagnostics):
# /tqbot/foot_position/fl  geometry_msgs/msg/Point
# /tqbot/foot_position/fr  geometry_msgs/msg/Point
# /tqbot/foot_position/rl  geometry_msgs/msg/Point
# /tqbot/foot_position/rr  geometry_msgs/msg/Point
# ----------------------------------------------------------------

# ----------------------------------------------------------------
# Imports
# ----------------------------------------------------------------
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray

from tqbot_kinematics import LegKinematics, Leg, DEFAULT_L1, DEFAULT_L2, DEFAULT_L3


# ----------------------------------------------------------------
# Leg configuration
# ----------------------------------------------------------------
LEG_CONFIGS = {
    "fl": Leg.FRONT_LEFT,
    "fr": Leg.FRONT_RIGHT,
    "rl": Leg.REAR_LEFT,
    "rr": Leg.REAR_RIGHT,
}


# ----------------------------------------------------------------
# Kinematics node
# ----------------------------------------------------------------
class KinematicsNode(Node):
    """
    ROS2 node that wraps LegKinematics for all four legs.
    """
    def __init__(self):
        super().__init__("tqbot_kinematics_node")
        # ------------- Declare and read params -------------
        self.declare_parameter("l1", DEFAULT_L1)
        self.declare_parameter("l2", DEFAULT_L2)
        self.declare_parameter("l3", DEFAULT_L3)

        l1 = self.get_parameter("l1").get_parameter_value().double_value
        l2 = self.get_parameter("l2").get_parameter_value().double_value
        l3 = self.get_parameter("l3").get_parameter_value().double_value

        self.get_logger().info(f"Kinematics node starting: "f"l1={l1:.4f} m, l2={l2:.4f} m, l3={l3:.4f} m")

        # ------------- Build LegKinematics per leg -------------
        self._kin = {
            name: LegKinematics(leg_enum, l1=l1, l2=l2, l3=l3)
            for name, leg_enum in LEG_CONFIGS.items()
        }

        # ------------- Publishers/subscribers per  leg -------------
        self._angle_pubs = {}
        self._fk_pubs = {}
        self._foot_subs = {}

        for name in LEG_CONFIGS:
            # ------------- Joint angle publisher -------------
            self._angle_pubs[name] = self.create_publisher(
                Float64MultiArray,
                f"/tqbot/joint_angles/{name}",
                10,
            )

            # ------------- FK publisher -------------
            self._fk_pubs[name] = self.create_publisher(
                Point,
                f"/tqbot/foot_position/{name}",
                10,
            )

            # ------------- Foot target subscriber -------------
            self._foot_subs[name] = self.create_subscription(
                Point,
                f"/tqbot/foot_target/{name}",
                self._make_callback(name),
                10,
            )

        self.get_logger().info("Kinematics node ready.")

    # ----------------------------------------------------------------
    # Callbacks
    # ----------------------------------------------------------------
    def _make_callback(self, leg_name: str):
        def callback(msg: Point):
            self._on_foot_target(leg_name, msg)
        return callback

    def _on_foot_target(self, leg_name: str, msg: Point):
        kin = self._kin[leg_name]

        try:
            t1, t2, t3 = kin.inverse(msg.x, msg.y, msg.z)
        except Exception as exc:
            self.get_logger().warn(
                f"[{leg_name.upper()}] IK failed for target "
                f"({msg.x:.4f}, {msg.y:.4f}, {msg.z:.4f}): {exc}"
            )
            return

        # ------------- Publish joint angles -------------
        angle_msg = Float64MultiArray()
        angle_msg.data = [t1, t2, t3]
        self._angle_pubs[leg_name].publish(angle_msg)

        # ------------- Publish FK -------------
        fx, fy, fz = kin.forward(t1, t2, t3)
        fk_msg = Point()
        fk_msg.x, fk_msg.y, fk_msg.z = fx, fy, fz
        self._fk_pubs[leg_name].publish(fk_msg)

        self.get_logger().debug(
            f"[{leg_name.upper()}] "
            f"target=({msg.x:.4f}, {msg.y:.4f}, {msg.z:.4f}) m  "
            f"angles=(θ1={t1:.4f}, θ2={t2:.4f}, θ3={t3:.4f}) rad  "
            f"fk=({fx:.4f}, {fy:.4f}, {fz:.4f}) m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()