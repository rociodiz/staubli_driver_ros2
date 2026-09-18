"""Carga y validación del fichero de configuracion de puntos de la trayectoria.

CAPA 1 DE 3 (configuracion): define QUÉ puntos debe visitar tool0.
El usuario solo edita 'config/points.yaml' (posición, orientación y tiempos);
el código no cambia al añadir/quitar puntos.

Formato soportado (point_type: cartesian):
    joint_names: [joint_1 .. joint_6]      # fijo, orden del controller
    point_type: cartesian                  # puntos cartesianos de tool0
    planning_frame: base_link              # frame en el que se expresan
    part_center_base_link: [x, y, z]       # centro de la pieza (modo "look_at")
    orientation_mode:
        - manual             -> cada punto debe fijar 'quaternion_xyzw'
        - look_at_part_center-> +Z de tool0 apunta al part_center_base_link
                               (cualquier punto puede sobreescribirlo con
                               'quaternion_xyzw')
    points:                                # cualquier número de puntos
      - name: ...
        position: [x, y, z]                # tool0 en base_link
        time_from_start: <s>               # instante absoluto, creciente
        quaternion_xyzw: [x, y, z, w]      # OPCIONAL (según orientation_mode)

También se mantiene point_type: joint (diagnóstico local) donde cada punto usa
'positions: [6 radianes]' y se omite la IK.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List

import yaml

EXPECTED_JOINTS = [f"joint_{i}" for i in range(1, 7)]
_ORIENTATION_MODES = ("manual", "look_at_part_center")


class PointsConfig:
    """Contenedor de la configuración de puntos y tolerancias del goal."""

    def __init__(self, config_path: str) -> None:
        self.config_path: Path = Path(config_path)
        if not self.config_path.is_file():
            raise FileNotFoundError(
                f"Configuracion no encontrada: {self.config_path}"
            )

        raw: Dict[str, Any] = yaml.safe_load(self.config_path.read_text())
        if not isinstance(raw, dict):
            raise ValueError("El fichero de configuracion debe ser un YAML de diccionario.")

        # --- Articulaciones (orden exacto del controller, no cambiar) ---
        self.joint_names: List[str] = raw.get("joint_names", [])
        if self.joint_names != EXPECTED_JOINTS:
            raise ValueError(
                f"'joint_names' debe ser {EXPECTED_JOINTS} "
                f"(orden del joint_trajectory_controller). Recibido: {self.joint_names}"
            )

        # --- Tipo de puntos ---
        self.point_type: str = raw.get("point_type", "cartesian")
        if self.point_type not in ("cartesian", "joint"):
            raise ValueError("point_type debe ser 'cartesian' o 'joint'.")

        # --- Frame de planificación ---
        self.planning_frame: str = raw.get("planning_frame", "base_link")

        # --- Orientación ---
        self.orientation_mode: str = raw.get("orientation_mode", "look_at_part_center")
        if self.orientation_mode not in _ORIENTATION_MODES:
            raise ValueError(
                f"orientation_mode debe ser {list(_ORIENTATION_MODES)}. "
                f"Recibido: {self.orientation_mode!r}"
            )
        center = raw.get("part_center_base_link", [0.0, 0.66, 0.24])
        if len(center) != 3:
            raise ValueError("'part_center_base_link' debe tener 3 valores [x, y, z].")
        self.part_center_base_link: List[float] = [float(v) for v in center]

        # --- Tolerancias del goal ---
        goal_tol = raw.get("goal_tolerance", {})
        self.goal_position_tolerance: float = float(goal_tol.get("position", 0.01))
        self.goal_velocity_tolerance: float = float(goal_tol.get("velocity", 0.05))
        self.goal_acceleration_tolerance: float = float(goal_tol.get("acceleration", 0.1))
        self.goal_time_tolerance: float = float(raw.get("goal_time_tolerance", 0.5))

        # Percolación de parámetros para la IK (semillas reproducibles).
        self.random_seed: int = int(raw.get("random_seed", 42))
        self.n_random_seeds: int = int(raw.get("n_random_seeds", 30))

        # --- Puntos (cualquier número) ---
        points = raw.get("points")
        if not isinstance(points, list) or len(points) == 0:
            raise ValueError("Debe definirse al menos un punto en 'points'.")
        self.points: List[Dict[str, Any]] = points

        self._validate()

    # ------------------------------------------------------------------
    def _validate(self) -> None:
        previous_t = None
        for i, p in enumerate(self.points):
            name = p.get("name", f"punto_{i}")

            t = p.get("time_from_start")
            if t is None:
                raise ValueError(f"{name}: falta 'time_from_start'.")
            t = float(t)
            if previous_t is not None and t <= previous_t:
                raise ValueError(
                    f"{name}: 'time_from_start' debe ser estrictamente creciente "
                    f"({previous_t} -> {t})."
                )
            previous_t = t

            if self.point_type == "cartesian":
                pos = p.get("position")
                jpos = p.get("positions")
                if pos is not None and jpos is not None:
                    raise ValueError(
                        f"{name}: no puede tener 'position' y 'positions' a la vez."
                    )
                if pos is not None:
                    if not isinstance(pos, list) or len(pos) != 3:
                        raise ValueError(
                            f"{name}: 'position' debe ser [x, y, z] en '{self.planning_frame}'."
                        )
                    q = p.get("quaternion_xyzw")
                    if q is not None and (not isinstance(q, list) or len(q) != 4):
                        raise ValueError(
                            f"{name}: 'quaternion_xyzw' debe ser [x, y, z, w] o faltar."
                        )
                    if self.orientation_mode == "manual" and q is None:
                        raise ValueError(
                            f"{name}: orientation_mode='manual' requiere 'quaternion_xyzw' "
                            f"en cada punto."
                        )
                else:
                    if not isinstance(jpos, list) or len(jpos) != 6:
                        raise ValueError(
                            f"{name}: 'positions' debe ser [q1..q6] con 6 radianes."
                        )
            elif self.point_type == "joint":
                positions = p.get("positions")
                if not isinstance(positions, list) or len(positions) != 6:
                    raise ValueError(
                        f"{name}: point_type='joint' requiere 'positions' con 6 valores."
                    )