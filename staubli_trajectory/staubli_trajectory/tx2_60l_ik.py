"""Modelo cinemático del Staubli TX2-60L basado en PyKDL.

Proporciona FK/IK de la cadena:
    base_link -> joint_1 -> ... -> joint_6 -> link_6 -> tool0

SIN depender de MoveIt ni de move_group (implementación directa con PyKDL).

FUENTE DE LOS PARÁMETROS (sin duplicarlos):
  La geometría se lee en tiempo de ejecución de los ficheros oficiales del
  repositorio 'staubli_robot_description/config/tx2_60l/':
    - kinematics.yaml  -> origen (xyz) y eje (axis) de cada articulación
    - robot_model.yaml -> offset del TCP tool0 respecto a link_6
    - joint_limits.yaml-> límites de posición (via tag YAML !degrees)
  De este modo, cualquier cambio en el modelo del robot se refleja aquí sin
  editar este módulo.

NOTA de construcción de la cadena KDL:
  En URDF cada 'origin' es la transformada desde el link padre al marco del
  joint, y la rotación ocurre en ese marco: T_i = Trans(origin) · Rot(axis,q).
  La semántica nativa de PyKDL es: Segment::pose(q) = Joint::pose(q) * f_tip,
  donde el constructor de Segment reescribe f_tip = Joint::pose(0)^-1 * f_tip_dado.
  Para obtener exactamente Trans(origin) · Rot(axis,q) hay que declarar el
  origin EN EL JOINT (Joint(name, origin, axis, RotAxis)), lo que compensado
  por esa reescritura deja Segment::pose(q) = Trans(origin) · Rot(axis,q).
  Con q = todas-0 el tool0 cae en (0, 0.02, 1.295) m (verificado).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np
import PyKDL as kdl
import yaml

# ----------------------------------------------------------------------------
# Carga de la geometría real del robot (única fuente de verdad).
# ----------------------------------------------------------------------------


class _RobotYamlLoader(yaml.SafeLoader):
    """SafeLoader con soporte del tag `!degrees` usado en joint_limits.yaml."""


def _degrees_constructor(loader: yaml.Loader, node: yaml.Node) -> float:
    return math.radians(float(loader.construct_scalar(node)))


_RobotYamlLoader.add_constructor("!degrees", _degrees_constructor)


def _load_yaml(path: Path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.load(f, Loader=_RobotYamlLoader)


def find_tx2_60l_config_dir() -> Path:
    """Localiza el directorio de configuración tx2_60l del robot.

    Prioridad:
      1. share instalado de 'staubli_robot_description' (entorno ROS fuenteado).
      2. árbol de fuentes del repositorio (permite ejecutar sin compilar).
    """
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("staubli_robot_description"))
        if (share / "config" / "tx2_60l").is_dir():
            return share / "config" / "tx2_60l"
    except Exception:  # noqa: BLE001 - ament_index no disponible sin ROS fuenteado
        pass

    # Fallback: subimos de 'staubli_trajectory' hasta encontrar el repo.
    here = Path(__file__).resolve()
    for parent in here.parents:
        cand = parent / "staubli_robot_description" / "config" / "tx2_60l"
        if cand.is_dir():
            return cand

    raise FileNotFoundError(
        "No se encuentra la configuracion tx2_60l del robot. ¿Está instalado "
        "'staubli_robot_description' o el repositorio de fuentes?"
    )


def load_tx2_60l_geometry(
    config_dir: Optional[Path] = None,
) -> Tuple[
    List[str],
    List[Tuple[float, float, float]],
    List[Tuple[float, float, float]],
    Tuple[float, float, float],
    List[Tuple[float, float]],
]:
    """Lee la geometría del TX2-60L de los YAML del repositorio.

    Devuelve (nombres_articulaciones, origenes_xyz, ejes_axis, tool0_xyz,
    limites_rad) con datos IDÉNTICOS a los del paquete de descripción.
    """
    config_dir = config_dir or find_tx2_60l_config_dir()

    kinematics = _load_yaml(config_dir / "kinematics.yaml")["kinematics"]
    robot_model = _load_yaml(config_dir / "robot_model.yaml")["robot_model"]
    limits = _load_yaml(config_dir / "joint_limits.yaml")["joint_limits"]

    names = [f"joint_{i}" for i in range(1, 7)]
    if set(kinematics) != set(names) or set(limits) != set(names):
        raise ValueError("El modelo tx2_60l no contiene joint_1..joint_6.")

    origins = [tuple(float(v) for v in kinematics[n]["xyz"]) for n in names]
    axes = [tuple(float(v) for v in kinematics[n]["axis"]) for n in names]

    ee = robot_model["end_effector"]
    tool0_xyz = tuple(float(v) for v in ee["xyz"])
    expected_rpy = (0.0, 0.0, 0.0)
    if tuple(float(v) for v in ee.get("rpy", [0.0] * 3)) != expected_rpy:
        raise ValueError("Se espera rpy=[0,0,0] para el TCP tool0 (según robot_model.yaml).")

    lim = [
        (float(limits[n]["min_position"]), float(limits[n]["max_position"]))
        for n in names
    ]
    return names, origins, axes, tool0_xyz, lim


# La geometría se carga una sola vez en import (fuente única de verdad).
_TX2_60L_NAMES, _TX2_60L_ORIGINS, _TX2_60L_AXES, _TOOL0_OFFSET, _JOINT_LIMITS_RAD = (
    load_tx2_60l_geometry()
)

# Tolerancias de validación del solver.
_IK_POS_EPS = 1.0e-4   # m
_IK_ROT_EPS = 1.0e-3   # rad


# ----------------------------------------------------------------------------
@dataclass
class IkResult:
    """Solución de IK de un punto.

    - q:              posiciones articulares [joint_1..joint_6] en rad.
    - solved:         True si el solver convergió dentro de tolerancia.
    - inside_limits:  True si q respeta todos los límites (con margen 1e-6).
    - joints_inside:  flag por articulación (mismo orden que q).
    - pos_error / rot_error: errores de la solución respecto al objetivo.
    - continuity:     distancia al waypoint anterior (ayuda a elegir rama).
    """

    q: List[float] = field(default_factory=lambda: [0.0] * 6)
    solved: bool = False
    inside_limits: bool = False
    joints_inside: List[bool] = field(default_factory=lambda: [True] * 6)
    pos_error: float = math.inf
    rot_error: float = math.inf
    continuity: float = 0.0


# ----------------------------------------------------------------------------
class Tx260lKinematics:
    """FK/IK del TX2-60L accesible sin lanzar move_group."""

    def __init__(self, random_seed: int = 42, n_random_seeds: int = 30) -> None:
        self.joint_names = list(_TX2_60L_NAMES)
        self._chain = self._build_chain()
        self._fk = kdl.ChainFkSolverPos_recursive(self._chain)
        self._ik = kdl.ChainIkSolverPos_LMA(self._chain, 1.0e-6, 500)
        self._rng = np.random.default_rng(random_seed)
        self._n_random_seeds = n_random_seeds

    # ------------------------------------------------------------------
    def _build_chain(self) -> kdl.Chain:
        """Construye la cadena KDL base_link -> joint_1..6 -> tool0."""
        chain = kdl.Chain()
        for name, origin, axis in zip(_TX2_60L_NAMES, _TX2_60L_ORIGINS, _TX2_60L_AXES):
            joint = kdl.Joint(name, kdl.Vector(*origin), kdl.Vector(*axis), kdl.Joint.RotAxis)
            chain.addSegment(kdl.Segment(joint, kdl.Frame(kdl.Vector(*origin))))
        # TCP fijo (tool0), hijo de link_6.
        chain.addSegment(
            kdl.Segment(kdl.Joint("tool0", kdl.Joint.Fixed), kdl.Frame(kdl.Vector(*_TOOL0_OFFSET)))
        )
        return chain

    # ------------------------------------------------------------------
    def forward_kinematics(self, q: Sequence[float]) -> kdl.Frame:
        """Posición/orientación de tool0 respecto a base_link para q."""
        jnt = kdl.JntArray(6)
        for i, v in enumerate(q):
            jnt[i] = float(v)
        frame = kdl.Frame()
        self._fk.JntToCart(jnt, frame)
        return frame

    # ------------------------------------------------------------------
    def inverse_kinematics(
        self,
        goal: kdl.Frame,
        q_prev: Optional[Sequence[float]] = None,
        q_state: Optional[Sequence[float]] = None,
    ) -> IkResult:
        """Resuelve la IK del frame 'goal' con arranque múltiple (LMA).

        El LMA de KDL puede converger a mínimos locales fuera de límites. Por
        eso se prueban varias semillas (ceros, estado actual, solución previa,
        perturbaciones de ésta y semillas aleatorias deterministas) y se elige:
          1. solución convergente (posición y rotación dentro de tolerancia),
          2. de entre ellas, la que cumpla los límites,
          3. y entre esas, la más continua respecto a q_prev (menor salto).
        """
        seeds: List[List[float]] = [[0.0] * 6]
        if q_state is not None:
            seeds.append([float(v) for v in q_state])
        if q_prev is not None:
            seeds.append([float(v) for v in q_prev])
            for _ in range(10):
                seeds.append(
                    [float(q_prev[i]) + self._rng.uniform(-0.05, 0.05) for i in range(6)]
                )
        for _ in range(self._n_random_seeds):
            lo, hi = zip(*_JOINT_LIMITS_RAD)
            seeds.append([self._rng.uniform(lo[i], hi[i]) for i in range(6)])

        candidatos: List[IkResult] = []
        vistas = set()  # deduplicar soluciones idénticas (misma rama)
        for seed in seeds:
            q_init = kdl.JntArray(6)
            for i, v in enumerate(seed):
                q_init[i] = v
            qout = kdl.JntArray(6)
            if self._ik.CartToJnt(q_init, goal, qout) < 0:
                continue
            q = [qout[i] for i in range(6)]
            clave = tuple(round(v, 3) for v in q)
            if clave in vistas:
                continue
            vistas.add(clave)

            res = self._check_solution(q, goal)
            if not res.solved:
                continue
            if q_prev is not None:
                res.continuity = sum((q[i] - float(q_prev[i])) ** 2 for i in range(6))
            else:
                res.continuity = 0.0
            candidatos.append(res)

        if not candidatos:
            return IkResult()

        # Orden: primero dentro de límites, luego menor salto respecto a q_prev.
        candidatos.sort(key=lambda r: (0 if r.inside_limits else 1, r.continuity))
        return candidatos[0]

    # ------------------------------------------------------------------
    def _check_solution(self, q: List[float], goal: kdl.Frame) -> IkResult:
        """Valida una solución por FK inversa (posición + orientación + límites)."""
        frame = self.forward_kinematics(q)
        res = IkResult(q=q)
        res.pos_error = (frame.p - goal.p).Norm()
        res.rot_error = abs((goal.M.Inverse() * frame.M).GetRotAngle()[0])
        res.solved = res.pos_error < _IK_POS_EPS and res.rot_error < _IK_ROT_EPS
        if res.solved:
            res.inside_limits, res.joints_inside = self.check_joint_limits(q)
        return res

    # ------------------------------------------------------------------
    def check_joint_limits(self, q: Sequence[float]) -> Tuple[bool, List[bool]]:
        """True si todas las articulaciones están dentro de sus límites (con margen)."""
        per: List[bool] = []
        for qj, (lo, hi) in zip(q, _JOINT_LIMITS_RAD):
            per.append(lo - 1.0e-6 <= float(qj) <= hi + 1.0e-6)
        return all(per), per

    # ------------------------------------------------------------------
    @staticmethod
    def orientation_frame_looking_at(
        position: Sequence[float], target: Sequence[float]
    ) -> kdl.Frame:
        """Frame con +Z de tool0 apuntando de 'position' hacia 'target'.

        Los ejes se obtienen así:
            z = normalize(target - position)      # eje de apuntado (+Z)
            aux =  Z salvo que z sea ~vertical     # referente estable
            y  = normalize(aux x z)
            x  = y x z                            # sistema ortonormal directo
            R  = [x | y | z] (columnas)           => R * [0,0,1] = z
        El giro alrededor del eje de apuntado queda libre (se fija de forma
        determinista). NO usa tf_transformations (no importable en este
        entorno): el quaternion se extrae con PyKDL.GetQuaternion() (x,y,z,w).
        """
        z = kdl.Vector(
            target[0] - position[0],
            target[1] - position[1],
            target[2] - position[2],
        )
        z = z * (1.0 / z.Norm())
        aux = kdl.Vector(1, 0, 0) if abs(z[2]) > 0.99 else kdl.Vector(0, 0, 1)
        y = aux * z                # cross(aux, z)
        y = y * (1.0 / y.Norm())
        x = y * z                  # cross(y, z)
        return kdl.Frame(kdl.Rotation(x, y, z), kdl.Vector(*position))

    # ------------------------------------------------------------------
    @staticmethod
    def frame_from_position_quaternion(
        position: Sequence[float], quaternion_xyzw: Sequence[float]
    ) -> kdl.Frame:
        """Frame a partir de posición y quaternion (x, y, z, w)."""
        q = kdl.Rotation.Quaternion(*[float(v) for v in quaternion_xyzw])
        return kdl.Frame(q, kdl.Vector(*position))

    # ------------------------------------------------------------------
    @staticmethod
    def quaternion_from_frame(frame: kdl.Frame) -> List[float]:
        """Devuelve el quaternion (x, y, z, w) del frame (orden de KDL/ROS)."""
        return list(frame.M.GetQuaternion())