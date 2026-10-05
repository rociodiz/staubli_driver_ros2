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

    def __init__(
        self,
        config: Optional[PointsConfig] = None,
        kinematics: Optional[Tx260lKinematics] = None,
    ):
        """`config` es opcional: quien ya trae la IK hecha puede usar
        `build_from_joint_waypoints()` sin pasar por `config/points.yaml`.

        Con `config=None` solo están disponibles las entradas que no leen la
        configuración; `build()` fallará con un mensaje claro.
        """
        self._config = config
        if kinematics is not None:
            self._kinematics = kinematics
        elif config is not None:
            self._kinematics = Tx260lKinematics(
                random_seed=config.random_seed,
                n_random_seeds=config.n_random_seeds,
            )
        else:
            self._kinematics = Tx260lKinematics()

    # ------------------------------------------------------------------
    def build(self) -> TrajectoryPlan:
        """Construye el plan: registros de diagnóstico + JointTrajectory."""
        if self._config is None:
            raise ValueError(
                "build() necesita un PointsConfig; para waypoints ya "
                "resueltos usa build_from_joint_waypoints()."
            )
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

    # ------------------------------------------------------------------
    def build_from_joint_waypoints(
        self,
        waypoints: Sequence[Dict[str, Any]],
        joint_names: Optional[Sequence[str]] = None,
    ) -> TrajectoryPlan:
        """TrajectoryPlan desde waypoints articulares YA resueltos (sin IK).

        Camino de entrada para quien ya tiene la IK hecha y no quiere que se
        vuelva a calcular: por ejemplo el barrido, donde `scan_path` resuelve
        P1/P2/P3 con el eje óptico real de la cámara y encadena la solución.
        Reutilizar ese `q` tal cual es lo que preserva la orientación de
        barrido; pasar los puntos por `build()` los reorientaría con
        `look_at_part_center` (que apunta el +Z de tool0, no el +X de la
        cámara) y podría elegir otra rama articular.

        No se usa `self._config`: los límites sí se verifican, con la misma
        cinemática, pero los puntos no salen del YAML. Si no se pasan
        `joint_names` explícitos, se toman de la configuración si la hay.

        Cada waypoint es un dict con:
          - `q`:                6 valores en radianes (obligatorio)
          - `time_from_start`:  segundos, absoluto y creciente (obligatorio)
          - `name`:             etiqueta para diagnóstico (opcional)
          - `velocities`:       6 valores; si se da, el controlador podrá usar
                                spline cúbico en vez de interpolación lineal
                                (no se usa en la v1 del barrido)

        El `JointTrajectory` resultante lleva `trajectory.frame_id` vacío, como
        el que ya genera `_to_joint_trajectory`.
        """
        if not waypoints:
            raise ValueError("se necesita al menos un waypoint")

        names = (
            list(joint_names)
            if joint_names
            else list(self._config.joint_names if self._config else [])
        )
        if not names:
            raise ValueError(
                "hace falta joint_names explícito: no hay PointsConfig de "
                "donde sacarlos"
            )
        n_joints = len(names)

        records: List[WaypointRecord] = []
        previous_time: Optional[float] = None

        for index, waypoint in enumerate(waypoints):
            q = [float(v) for v in waypoint["q"]]
            if len(q) != n_joints:
                raise ValueError(
                    f"waypoint {index} ({waypoint.get('name', '?')}): {len(q)} "
                    f"articulaciones, se esperaban {n_joints}"
                )
            inside, per_joint = self._kinematics.check_joint_limits(q)

            time = float(waypoint["time_from_start"])
            if previous_time is not None and time <= previous_time:
                raise ValueError(
                    f"waypoint {index} ({waypoint.get('name', '?')}): "
                    f"time_from_start={time} no es mayor que el anterior "
                    f"({previous_time}); los tiempos deben ser crecientes"
                )
            previous_time = time

            records.append(
                WaypointRecord(
                    name=str(waypoint.get("name", f"waypoint_{index + 1}")),
                    q=q,
                    joint_ok=per_joint,
                    inside_limits=inside,
                    solved=True,
                    info=waypoint.get("info", "q precomputed (sin IK)"),
                )
            )

        plan = TrajectoryPlan(records=records)
        plan.all_solved = True
        plan.all_inside_limits = all(r.inside_limits for r in records)
        if plan.all_inside_limits:
            plan.trajectory = self._joint_trajectory_from_waypoints(waypoints, names)
        return plan

    # ------------------------------------------------------------------
    @staticmethod
    def _joint_trajectory_from_waypoints(
        waypoints: Sequence[Dict[str, Any]], joint_names: Sequence[str]
    ) -> JointTrajectory:
        """Construye el JointTrajectory desde los dicts de waypoint.

        Gemelo de `_to_joint_trajectory()` pero sin `PointsConfig`: por eso no
        lleva velocidades (el JTC interpola linealmente) y por eso los
        tiempos salen del propio waypoint.
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = list(joint_names)
        trajectory.points = []
        for waypoint in waypoints:
            msg_point = JointTrajectoryPoint()
            msg_point.positions = [float(v) for v in waypoint["q"]]
            if waypoint.get("velocities"):
                msg_point.velocities = [float(v) for v in waypoint["velocities"]]
            msg_point.time_from_start = Duration(
                seconds=float(waypoint["time_from_start"])
            ).to_msg()
            trajectory.points.append(msg_point)
        return trajectory