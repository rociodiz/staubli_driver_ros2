"""Nodo de control cartesiano por velocidad del TX2-60L (sin MoveIt).

Flujo (frame de control: base_link):

    /cmd_vel (geometry_msgs/Twist)
        -> deadzone por eje
        -> límites cartesianos globales (lineal/angular)
        -> q_dot = J^T (J J^T + lambda^2 I)^-1 Twist   (DLS + SVD)
        -> límites de velocidad articulares (escalado proporcional)
        -> rampas de aceleración/desaceleración
        -> /forward_velocity_controller/commands (Float64MultiArray, joint_1..6)

Seguridad:
    - Arranca publicando ceros y lo mantiene hasta tener /joint_states.
    - Sin /joint_states recientes (joint_state_timeout) -> ceros inmediatos.
    - Pérdida de /cmd_vel (command_timeout) -> ceros inmediatos (watchdog).
    - Twist dentro de deadzone -> objetivo cero (frenado progresivo).
    - Cerca de singularidad -> reduce Twist y aumenta amortiguamiento.
    - El límite articular (J^T...) nunca publica velocidades explotadas.

La cinemática (cadena KDL, FK, Jacobiano 6x6 de tool0 en base_link) se reutiliza
de staubli_trajectory.tx2_60l_ik.Tx260lKinematics: no se duplica geometría.
"""

from __future__ import annotations

from typing import List, Optional, Sequence

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from staubli_trajectory.tx2_60l_ik import JOINT_VELOCITY_LIMITS_RAD, Tx260lKinematics

from .velocity_limiter import (
    JointVelocityLimiter,
    apply_deadzone,
    clamp_vector,
    scale_to_joint_velocity_limits,
    solve_qdot_dls,
)

_JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]


class JoystickServoNode(Node):
    """Convierte /cmd_vel cartesiano en velocidades articulares."""

    def __init__(self) -> None:
        super().__init__("staubli_joystick_servo")
        self.declare_parameter("control_rate", 250.0)
        self.declare_parameter("command_timeout", 0.2)
        self.declare_parameter("joint_state_timeout", 0.2)
        self.declare_parameter("damping_lambda", 0.05)
        self.declare_parameter("singularity_threshold", 0.02)
        self.declare_parameter("max_linear_speed", 0.15)
        self.declare_parameter("max_angular_speed", 0.5)
        self.declare_parameter("acceleration_limit", 1.5)
        self.declare_parameter("deadzone_linear", 0.01)
        self.declare_parameter("deadzone_angular", 0.01)

        self._control_rate = float(self.get_parameter("control_rate").value)
        self._command_timeout = float(self.get_parameter("command_timeout").value)
        self._joint_state_timeout = float(self.get_parameter("joint_state_timeout").value)
        self._damping_lambda = float(self.get_parameter("damping_lambda").value)
        self._singularity_threshold = float(self.get_parameter("singularity_threshold").value)
        self._max_linear = float(self.get_parameter("max_linear_speed").value)
        self._max_angular = float(self.get_parameter("max_angular_speed").value)
        self._deadzone_lin = float(self.get_parameter("deadzone_linear").value)
        self._deadzone_ang = float(self.get_parameter("deadzone_angular").value)

        self._joint_velocity_limits = list(JOINT_VELOCITY_LIMITS_RAD)
        self._kinematics = Tx260lKinematics()
        self._limiter = JointVelocityLimiter(
            self._joint_velocity_limits,
            float(self.get_parameter("acceleration_limit").value),
            self._control_rate,
        )

        # Estado entrante.
        self._latest_twist: Optional[Twist] = None
        self._latest_twist_t: Optional[rclpy.time.Time] = None
        self._joint_positions: Optional[List[float]] = None
        self._joint_state_t: Optional[rclpy.time.Time] = None

        # ROS 2.
        self._cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, 10
        )
        self._joint_states_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10
        )
        self._commands_pub = self.create_publisher(
            Float64MultiArray, "/forward_velocity_controller/commands", 10
        )

        period = 1.0 / self._control_rate
        self._timer = self.create_timer(period, self._on_timer)
        self.get_logger().info(
            f"Servo listo: rate={self._control_rate:.0f} Hz | "
            f"joint limits={[round(v, 3) for v in self._joint_velocity_limits]}"
        )

    # ------------------------------------------------------------------
    # Callbacks de entrada.
    # ------------------------------------------------------------------

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._latest_twist = msg
        self._latest_twist_t = self.get_clock().now()

    def _on_joint_states(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in _JOINT_NAMES):
            self._joint_positions = [float(positions[name]) for name in _JOINT_NAMES]
            self._joint_state_t = self.get_clock().now()

    # ------------------------------------------------------------------
    # Bucle de control.
    # ------------------------------------------------------------------

    def _publish_commands(self, values: Sequence[float]) -> None:
        msg = Float64MultiArray()
        msg.data = [float(v) for v in values]
        self._commands_pub.publish(msg)

    def _elapsed(self, stamp: Optional[rclpy.time.Time]) -> Optional[float]:
        if stamp is None:
            return None
        now = self.get_clock().now()
        return (now - stamp).nanoseconds * 1.0e-9

    def _on_timer(self) -> None:
        zeros = [0.0] * 6

        # 1) Requiere /joint_states frescos.
        elapsed_js = self._elapsed(self._joint_state_t)
        if elapsed_js is None or elapsed_js > self._joint_state_timeout:
            self._limiter.reset()
            self._publish_commands(zeros)
            return

        # 2) Watchdog de /cmd_vel: pérdida de comando -> STOP inmediato.
        elapsed_cmd = self._elapsed(self._latest_twist_t)
        if elapsed_cmd is None or elapsed_cmd > self._command_timeout:
            self._publish_commands(self._limiter.force_zero())
            return

        # 3) Deadzone por eje del Twist.
        twist = self._latest_twist
        lin, ang = apply_deadzone(
            (twist.linear.x, twist.linear.y, twist.linear.z),
            (twist.angular.x, twist.angular.y, twist.angular.z),
            self._deadzone_lin,
            self._deadzone_ang,
        )

        # Twist nulo (o dentro de deadzone) -> frenado progresivo a cero.
        if all(abs(v) < 1.0e-12 for v in lin + ang):
            target = np.zeros(6)
        else:
            lin = clamp_vector(lin, self._max_linear)
            ang = clamp_vector(ang, self._max_angular)
            twist_vec = np.array([*lin, *ang], dtype=float)

            jac = self._kinematics.jacobian(self._joint_positions)
            q_dot, sigma_min, _, _ = solve_qdot_dls(
                jac, twist_vec, self._damping_lambda, self._singularity_threshold
            )
            target = scale_to_joint_velocity_limits(q_dot, self._joint_velocity_limits)

        out = self._limiter.update(target)
        self._publish_commands(out)

    # ------------------------------------------------------------------
    # STOP manual (puede invocarse desde launch o como servicio futuro).
    # ------------------------------------------------------------------

    def stop(self) -> None:
        self._publish_commands(self._limiter.force_zero())


# ---------------------------------------------------------------------------
def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = JoystickServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()