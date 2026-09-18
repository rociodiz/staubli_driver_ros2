"""Cliente de accion para enviar trayectorias al joint_trajectory_controller.

CAPA 3 DE 3 (envio): ActionClient del control_msgs/action/FollowJointTrajectory.

Action server (verificado en el repositorio):
    /joint_trajectory_controller/follow_joint_trajectory

Contrato del controlador (staubli_bringup/config/controllers.yaml):
    - joints: joint_1 .. joint_6   (orden exacto)
    - command_interfaces: position (todas las posiciones van en radianes)
"""

from __future__ import annotations

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectory

from .points_config import PointsConfig

# Codigos de resultado del FollowJointTrajectoryResult (verificado en .action).
_RESULT_CODES = {
    FollowJointTrajectory.Result.SUCCESSFUL: "SUCCESSFUL",
    FollowJointTrajectory.Result.INVALID_GOAL: "INVALID_GOAL",
    FollowJointTrajectory.Result.INVALID_JOINTS: "INVALID_JOINTS",
    FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP: "OLD_HEADER_TIMESTAMP",
    FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED: "PATH_TOLERANCE_VIOLATED",
    FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED: "GOAL_TOLERANCE_VIOLATED",
}


class JointTrajectoryClient:
    """Envia un JointTrajectory al action server del joint_trajectory_controller.

    Uso tipico:
        client = JointTrajectoryClient(node)
        if client.wait_for_server():
            client.send(trajectory, config)
    """

    ACTION_NAME = "/joint_trajectory_controller/follow_joint_trajectory"

    def __init__(self, node: Node) -> None:
        self._node = node
        self._client = ActionClient(
            node, FollowJointTrajectory, self.ACTION_NAME
        )

    # ------------------------------------------------------------------
    def wait_for_server(self, timeout_s: float = 10.0) -> bool:
        """Espera a que el action server este disponible (controllers activos)."""
        if not self._client.wait_for_server(timeout_sec=timeout_s):
            self._node.get_logger().error(
                f"Action server '{self.ACTION_NAME}' no disponible tras {timeout_s} s. "
                "¿Está lanzado el Gazebo/robot y activado el joint_trajectory_controller?"
            )
            return False
        self._node.get_logger().info(f"Action server '{self.ACTION_NAME}' disponible.")
        return True

    # ------------------------------------------------------------------
    def send(self, trajectory: JointTrajectory, config: PointsConfig) -> bool:
        """Construye el goal, lo envia y espera el resultado ejecutado.

        Devuelve True si la trayectoria termina en SUCCESSFUL.
        """
        goal = self._build_goal(trajectory, config)

        self._node.get_logger().info(
            f"Enviando goal con {len(goal.trajectory.points)} waypoints "
            f"(joint_1..joint_6 en rad): "
            f"{[p.positions for p in goal.trajectory.points]}"
        )

        # Envio asíncrono; se espera con spin_until_future_complete (un solo ejecutor).
        goal_future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        rclpy.spin_until_future_complete(self._node, goal_future)
        if not goal_future.done():
            self._node.get_logger().error("Envío del goal incompleto (timeout).")
            return False

        intento_goal = goal_future.result()
        if not intento_goal.accepted:
            self._node.get_logger().error("Goal RECHAZADO por el controller.")
            return False

        self._node.get_logger().info("Goal ACEPTADO por el controller; ejecutando...")
        result_future = intento_goal.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)
        if not result_future.done():
            self._node.get_logger().error("Ejecución en curso sin resultado (timeout).")
            return False

        result = result_future.result()
        resultado = getattr(result, "result", result)
        error_code = getattr(resultado, "error_code", None)
        nombre = _RESULT_CODES.get(error_code, f"desconocido ({error_code})")

        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self._node.get_logger().info(f"Trayectoria completada con éxito ({nombre}).")
            return True

        self._node.get_logger().error(
            f"Trayectoria finalizada con código {nombre}. "
            f"Detalle: {getattr(resultado, 'error_string', '')}"
        )
        return False

    # ------------------------------------------------------------------
    @staticmethod
    def _build_goal(
        trajectory: JointTrajectory, config: PointsConfig
    ) -> FollowJointTrajectory.Goal:
        """Construye el goal: la trayectoria + tolerancias por articulación."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        # Tolerancia final por articulación (JointTolerance).
        for name in config.joint_names:
            tol = JointTolerance()
            tol.name = name
            tol.position = config.goal_position_tolerance
            tol.velocity = config.goal_velocity_tolerance
            tol.acceleration = config.goal_acceleration_tolerance
            goal.goal_tolerance.append(tol)

        # Leeway temporal para declarar éxito en el punto final.
        goal.goal_time_tolerance = Duration(
            seconds=config.goal_time_tolerance
        ).to_msg()
        return goal

    # ------------------------------------------------------------------
    @staticmethod
    def _on_feedback(feedback_msg) -> None:
        """Feedback del controller durante la ejecución (se puede loguear)."""
        fb = getattr(feedback_msg, "feedback", feedback_msg)
        idx = getattr(fb, "index", -1)
        actual = getattr(getattr(fb, "actual", None), "positions", [])
        if actual:
            mostra = ", ".join(f"{v:.3f}" for v in actual)
            print(f"[feedback] waypoint {idx}: actual=[{mostra}]")