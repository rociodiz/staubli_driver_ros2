"""Calculo de la trayectoria articulada a partir de los puntos cartesianos.

CAPA 2 DE 3 (calculo): convierte los puntos de config/points.yaml en una
trayectoria articulada para el joint_trajectory_controller:

    position (XYZ, base_link) + orientación  ->  frame tool0  ->  IK (PyKDL)
    ->  q1..q6  ->  verificación de límites  ->  FollowJointTrajectory

Separación de responsabilidades:
  1. points_config    -> QUÉ puntos visitar
  2. TrajectoryBuilder-> CÓMO viajar (cartesianos -> articulaciones, con IK)
  3. joint_trajectory_client -> ENVÍO del goal al controller

Orientación de cada punto:
  - Si el punto define 'quaternion_xyzw' -> se usa tal cual (manual).
  - Si no, y orientation_mode='look_at_part_center' -> el eje +Z de tool0 se
    calcula para apuntar al 'part_center_base_link' (futuro/actual de inspección).

El calculo es totalmente independiente de MoveIt / move_group.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence

from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .points_config import PointsConfig
from .tx2_60l_ik import Tx260lKinematics


@dataclass
class WaypointRecord:
    """Diagnóstico por punto (se imprime en terminal y se reutiliza al enviar)."""

    name: str = "waypoint"
    cartesian_position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    quaternion_xyzw: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])
    q: List[float] = field(default_factory=lambda: [0.0] * 6)
    joint_ok: List[bool] = field(default_factory=lambda: [True] * 6)
    inside_limits: bool = True
    solved: bool = False
    info: str = ""


@dataclass
class TrajectoryPlan:
    """Resultado del cálculo: registros por punto + la trayectoria lista."""

    records: List[WaypointRecord] = field(default_factory=list)
    trajectory: Optional[JointTrajectory] = None
    all_solved: bool = False
    all_inside_limits: bool = False


class TrajectoryBuilder:
    """Convierte PointsConfig en una JointTrajectory para el controller."""

    def __init__(self, config: PointsConfig, kinematics: Optional[Tx260lKinematics] = None):
        self._config = config
        self._kinematics = kinematics or Tx260lKinematics(
            random_seed=config.random_seed,
            n_random_seeds=config.n_random_seeds,
        )

    # ------------------------------------------------------------------
    def build(self) -> TrajectoryPlan:
        """Construye el plan: registros de diagnóstico + JointTrajectory."""
        if self._config.point_type == "joint":
            return self._build_joint_mode()

        records: List[WaypointRecord] = []
        q_prev: Optional[Sequence[float]] = None

        for point in self._config.points:
            record = self._solve_point(point, q_prev)
            records.append(record)
            if record.solved:
                q_prev = record.q
            else:
                q_prev = None  # no encadenar soluciones fallidas

        plan = TrajectoryPlan(records=records)
        plan.all_solved = all(r.solved for r in records)

        if plan.all_solved:
            plan.trajectory = self._to_joint_trajectory(records)
            plan.all_inside_limits = all(r.inside_limits for r in records)
        return plan

    # ------------------------------------------------------------------
    def _solve_point(
        self, point: Dict[str, Any], q_prev: Optional[Sequence[float]]
    ) -> WaypointRecord:
        """Resuelve la IK de un punto cartesiano y devuelve el registro."""
        name = point.get("name", "waypoint")

        # Punto articular directo (override): se usa 'positions' tal cual, sin IK.
        # Se evalúa primero para no romper el flujo cartesiano de los demás puntos.
        if "positions" in point:
            q = [float(v) for v in point["positions"]]
            ok, per = self._kinematics.check_joint_limits(q)
            return WaypointRecord(
                name=name,
                q=q,
                joint_ok=per,
                inside_limits=ok,
                solved=True,
                info="articular (passthrough, sin IK)",
            )

        position = [float(v) for v in point["position"]]
        record = WaypointRecord(name=name, cartesian_position=position)

        # --- Orientación de tool0 ---
        if "quaternion_xyzw" in point:
            # Orientación explícita (manual/override).
            frame = self._kinematics.frame_from_position_quaternion(
                position, point["quaternion_xyzw"]
            )
            record.quaternion_xyzw = [float(v) for v in point["quaternion_xyzw"]]
            record.info = "orientación manual"
        elif self._config.orientation_mode == "look_at_part_center":
            # +Z de tool0 apunta al centro de la pieza (cálculo automático).
            frame = self._kinematics.orientation_frame_looking_at(
                position, self._config.part_center_base_link
            )
            record.quaternion_xyzw = self._kinematics.quaternion_from_frame(frame)
            record.info = "+Z -> centro pieza"
        else:  # pragma: no cover - ya validado en PointsConfig
            raise ValueError(
                f"{name}: falta 'quaternion_xyzw' (orientation_mode='manual')."
            )

        # --- IK ---
        result = self._kinematics.inverse_kinematics(
            frame, q_prev=q_prev, q_state=None
        )

        record.solved = result.solved
        record.q = result.q
        record.joint_ok = result.joints_inside
        record.inside_limits = result.inside_limits

        if not result.solved:
            record.info = f"IK SIN SOLUCIÓN (pos {result.pos_error:.4f} m, rot {result.rot_error:.4f} rad)"
        elif not result.inside_limits:
            record.info = "IK ok pero FUERA DE LÍMITES"
        return record

    # ------------------------------------------------------------------
    def _build_joint_mode(self) -> TrajectoryPlan:
        """Modo passthrough (point_type: joint) para diagnóstico sin IK."""
        records: List[WaypointRecord] = []
        for point in self._config.points:
            q = [float(v) for v in point["positions"]]
            ok, per = self._kinematics.check_joint_limits(q)
            records.append(
                WaypointRecord(
                    name=point.get("name", "waypoint"),
                    q=q,
                    joint_ok=per,
                    inside_limits=ok,
                    solved=True,
                    info="modo joint (passthrough)",
                )
            )
        plan = TrajectoryPlan(records=records)
        plan.all_solved = True
        plan.all_inside_limits = all(r.inside_limits for r in records)
        plan.trajectory = self._to_joint_trajectory(records)
        return plan

    # ------------------------------------------------------------------
    def _to_joint_trajectory(self, records: List[WaypointRecord]) -> JointTrajectory:
        """Construye el mensaje trajectory_msgs/JointTrajectory."""
        trajectory = JointTrajectory()
        trajectory.joint_names = self._config.joint_names
        trajectory.points = []
        for (point, record) in zip(self._config.points, records):
            msg_point = JointTrajectoryPoint()
            msg_point.positions = [float(v) for v in record.q]
            msg_point.time_from_start = Duration(
                seconds=float(point["time_from_start"])
            ).to_msg()
            trajectory.points.append(msg_point)
        return trajectory