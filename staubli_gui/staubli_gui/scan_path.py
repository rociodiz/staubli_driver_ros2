"""Geometría de barrido y persistencia de los puntos P1/P2 sobre un cubo.

Un `ScanPathSpec` describe UN barrido lineal sobre UNA de las seis caras de un
cubo. La fuente de verdad es geométrica y deliberadamente mínima:

    part:           centro y lado del cubo, en base_link
    scan_distance:  distancia del SENSOR al PLANO DE APOYO del cubo, en metros
    scan_paths[]:   nombre, cara, longitud y sentido de la línea

`scan_distance` se mide desde el plano sobre el que se apoya el cubo —la mesa
en la celda de inspección—, NO desde su cara superior. La pieza sobresale por
encima de ese plano y es precisamente el volumen que la cámara debe capturar,
así que tomar la cara superior como referencia dejaría la cámara demasiado
baja para barrerla.

**P1/P2 son posiciones del SENSOR** (`sensor_support_link`), no de tool0. Ésa
es la corrección de la fase 2: antes se situaba tool0 en el punto de barrido y,
como el soporte va montado ~169 mm por delante de la brida a lo largo del
boresight, el sensor quedaba en realidad esa distancia más lejos de la
superficie de lo que decía la tabla. El cálculo `tool0 = sensor · montaje⁻¹` y
la medición de `distance_measured` desde el ORIGEN del sensor (vía FK) viven en
`frame_sensor_on_surface()` y `resolve_scan()`.

Los puntos P1/P2 y su validación NO se guardan: son DERIVADOS. Se recalculan
con `ScanPathStore.resolve()` a partir de la geometría y de la IK real del
TX2-60L, de forma que cambiar `scan_distance` no deja residuos obsoletos en
disco y dos ejecuciones del mismo perfil dan exactamente el mismo barrido.

Convención de orientación
-------------------------
El sensor es eye-in-hand (`staubli_camera.xacro`), pero el FOV que define la
geometría de barrido es el cono visual de `sensor_support_link`: el triángulo
translúcido y la línea roja cuelgan de ESE link, no de `camera_link` (`camera_link`
lleva su propio rpy de montaje y su +X está ~19° fuera del boresight del FOV).
El boresight del FOV es el +X del soporte, que coincide con el **+Z de tool0**
(`sensor_support_joint` rpy `0 -π/2 π/2`); el soporte Y es la normal del plano
del FOV y el soporte Z la dirección de la línea roja.

Por eso los puntos se orientan de modo que ese +X del soporte (boresight del
FOV, +Z de tool0) apunte a la superficie. El roll de tool0 lo fija la
convención histórica

    R_tool0 = R_FOV · R_montaje⁻¹

con `R_montaje = RPY(0,-π/2,0)`, elegido para que la línea roja quede
perpendicular al avance. Sobre esa orientación se RETROCEDE tool0 el vector de
montaje FÍSICO para que el sensor, y no la brida, quede en la posición pedida.

`tool0` está fijo a `link_6` en `xyz="0 0 0.07"` sin rpy.

Consecuencia medida de esta convención: apuntando el +X del soporte a la cara,
el plano del FOV queda perfectamente perpendicular a la superficie y la muñeca
(`joint_5`) deja de ser el eslabón crítico de la cara superior. Al colocar el
SENSOR (no tool0) a la distancia pedida, +Z (la cota de trabajo, 0.325 m desde
la mesa) se alcanza holgada y aguanta hasta ~0.50 m; más allá cae por joint_5.
En cambio caras en las que el sensor debe apuntar hacia el robot con la pieza
de por medio (-Y, con la parte ya acercada a 0.20 m del robot) dejan de
resolver. `resolve()` lo reporta en `ScanPoint.offending_joints` y
`ScanPoint.solved` en vez de ocultarlo, para que quien use esto distinga "fuera
de límites" de "no converge".

Nota sobre determinismo: `Tx260lKinematics` es multi-start y sortea sus
semillas de un `random.Random` cuyo estado avanza en cada llamada, así que
"resuelve fuera de límites" y "no converge" pueden alternarse para un mismo
objetivo según cuántas llamadas de IK se hayan hecho antes. Con una instancia
nueva por barrido el veredicto es reproducible.

Inclinación del boresight (caras laterales)
-------------------------------------------
La orientación ideal apunta el boresight perpendicularmente a la cara. Para
las caras laterales (±X, ±Y) esa pose puede quedar fuera de los límites
articulares. En ese caso `resolve_scan()` prueba una inclinación creciente del
boresight respecto a la normal (`max_tilt_deg`) y un giro alrededor de la
normal, y se queda con la primera inclinación que resuelve P1 y P2 dentro de
los límites, con margen a los límites blandos y sin un salto articular grande.
El barrido `+Z` (la cota de trabajo) conserva siempre la pose perpendicular
original; `-Z` tampoco se inclina porque su cara está sobre la mesa.

La colocación es "rayo": el sensor se sitúa a `scan_distance` de la cara a lo
largo del boresight, de modo que `distance_measured` sigue siendo la distancia
AXIAL de trabajo. La distancia perpendicular al plano es
`scan_distance * cos(tilt)`.

Puntos
------
    P1  inicio de la línea de barrido
    P2  final de la línea de barrido    (P1 → P2 es el único segmento)

No hay punto de retirada: el barrido termina en P2.

Persistencia (independiente de config/points.yaml):

    $XDG_CONFIG_HOME/staubli_gui/scan_paths.yaml

Reutiliza el directorio de `user_points` (override con
STAUBLI_GUI_CONFIG_DIR) y escribe de forma atómica (temp + os.replace).
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import PyKDL as kdl
import yaml

from .user_points import default_config_dir


# ─── Filtro geométrico aproximado centralizado ───────────────────────────
# Delegado a collision_common (sensor vs mesa/pieza). Solo centraliza, no
# generaliza a todo el robot.
from . import collision_common

TABLE_TOP_Z_BASELINK = collision_common.TABLE_TOP_Z_BASELINK
TABLE_HALF_X = collision_common.TABLE_HALF_X
TABLE_HALF_Y = collision_common.TABLE_HALF_Y
TABLE_CENTER_Y = collision_common.TABLE_CENTER_Y
TABLE_CLEARANCE_M = collision_common.TABLE_CLEARANCE_M
SENSOR_COLLISION_LOCAL_MIN = collision_common.SENSOR_COLLISION_LOCAL_MIN
SENSOR_COLLISION_LOCAL_MAX = collision_common.SENSOR_COLLISION_LOCAL_MAX


def _aabb_intersect(a_min, a_max, b_min, b_max, margin: float = 0.0) -> bool:
    return collision_common._aabb_intersect(a_min, a_max, b_min, b_max, margin)


def _sensor_oriented_aabb(sensor_frame_world: kdl.Frame):
    return collision_common.sensor_oriented_aabb(sensor_frame_world)


def _table_aabb():
    return collision_common.table_aabb()


def _part_aabb(part: PartBox):
    return collision_common.part_aabb(part)


def check_sensor_environment_collision(sensor_frame_world: kdl.Frame, part: PartBox, clearance: float = TABLE_CLEARANCE_M) -> Tuple[bool, str]:
    """Comprueba colisión aproximada sensor vs mesa/pieza (delegado a collision_common)."""
    return collision_common.check_sensor_environment_collision(sensor_frame_world, part, clearance)

# ─── Montaje del sensor/FOV (espejo de staubli_camera.xacro) ──────────────
#
# NO se lee el xacro en runtime: son valores fijos de la descripción del
# robot, y releer un URDF por cada barrido es I/O innecesario. Se replican
# aquí para que la geometría sea testeable sin lanzar Gazebo.
#
# El cono FOV (triángulo translúcido + línea roja) cuelga de
# `sensor_support_link`, no de `camera_link`. Su boresight es el +X del
# soporte, que en tool0 coincide con el +Z (`sensor_support_joint` rpy
# `0 -π/2 π/2`). Ejes del soporte en tool0:
#
#     +X (boresight del FOV) → +Z de tool0
#     +Z (dirección de la línea roja) → -Y de tool0
#
# La orientación desea el boresight apuntando a la superficie y la línea roja
# perpendicular al avance. Se materializa con un único rpy de montaje virtual
# RPY(0, -π/2, 0) = Ry(-π/2):
#   1) lleva el +X de la frame de apuntado a +Z de tool0 (el boresight mira a
#      la cara);
#   2) deja el +Z de la frame (la "línea") en -X de tool0, perpendicular al
#      eje de avance [0,1,0] de la celda (la perpendicularidad no depende del
#      signo del avance).
#
# Todo ello en RADIANES (como <origin rpy> del URDF); el sufijo _DEG del
# nombre es histórico y engañoso.

CAMERA_MOUNT_RPY_DEG: Tuple[float, float, float] = (0.0, -math.pi / 2.0, 0.0)
TOOL0_MOUNT_RPY_DEG: Tuple[float, float, float] = (0.0, 0.0, 0.0)

# ─── Montaje FÍSICO del soporte del sensor (sensor_support_link) ─────────
#
# Espejo del joint `sensor_support_joint` de staubli_camera.xacro:
#
#     <origin xyz="-0.003110 0.016130 0.168690" rpy="0 -1.570796 1.570796"/>
#
# NO es lo mismo que `CAMERA_MOUNT_RPY_DEG`, que es un rpy VIRTUAL elegido
# sólo para fijar el roll de tool0 (deja la línea roja del FOV perpendicular
# al avance). Éste es el montaje REAL: sensor_support_link +X = tool0 +Z (el
# boresight), de modo que situar el SENSOR a `scan_distance` del plano de
# apoyo exige RETROCEDER tool0 el vector de montaje (168.69 mm a lo largo del
# boresight). Antes se colocaba tool0 en el punto de barrido, lo que dejaba el
# sensor realmente ~169 mm más lejos de la superficie de lo que decía la tabla.
SENSOR_FRAME: str = "sensor_support_link"
SENSOR_MOUNT_XYZ: Tuple[float, float, float] = (-0.003110, 0.016130, 0.168690)
SENSOR_MOUNT_RPY: Tuple[float, float, float] = (0.0, -math.pi / 2.0, math.pi / 2.0)

# ─── Superficie: las seis caras del cubo ─────────────────────────────────

FACE_NORMALS: Dict[str, Tuple[float, float, float]] = {
    "+Z": (0.0, 0.0, 1.0),
    "-Z": (0.0, 0.0, -1.0),
    "+X": (1.0, 0.0, 0.0),
    "-X": (-1.0, 0.0, 0.0),
    "+Y": (0.0, 1.0, 0.0),
    "-Y": (0.0, -1.0, 0.0),
}

# Cara opuesta a cada cara: la que queda en el plano de apoyo.
OPPOSITE_FACE: Dict[str, str] = {
    "+Z": "-Z", "-Z": "+Z",
    "+X": "-X", "-X": "+X",
    "+Y": "-Y", "-Y": "+Y",
}

# La única cara cuyo plano de apoyo es la MESA. La pieza apoya sobre su cara
# -Z, así que al barrer +Z la distancia se mide desde la mesa; para el resto de
# caras la referencia sigue siendo la propia cara.
TABLE_BORNE_FACE: str = "+Z"

ROLES: Tuple[str, str] = ("P1", "P2")

# Parte por defecto de la celda de inspección, en base_link. Es el valor que
# se usa si `scan_paths.yaml` no declara `part`, para que la GUI arranque con
# la geometría validada en lugar de un cubo en el origen. La pieza mundial está
# en (0.0, 0.46, 0.54) y base_link se desplaza (0, 0, 0.30) respecto al mundo
# (spawn de Gazebo), así que el centro en base_link es (0.0, 0.46, 0.24).
DEFAULT_PART_CENTER: List[float] = [0.0, 0.46, 0.24]
DEFAULT_PART_SIZE: List[float] = [0.08, 0.08, 0.08]

# Distancia de trabajo inicial, en metros: la distancia óptima del sensor a la
# superficie inspeccionada. Es configurable (GUI/scan_paths.yaml); este es solo
# el valor de arranque, en sintonía con
# `sensor_reference.DEFAULT_WORKING_DISTANCE_M` (la referencia verde). Los dos
# módulos son independientes a propósito: scan_path describe la geometría de la
# trayectoria y sensor_reference la del sensor. Un test comprueba que no
# divergen.
DEFAULT_SCAN_DISTANCE: float = 0.325

# Inclinación máxima del boresight respecto a la normal de la cara. Se aplica
# sólo a las caras laterales (±X, ±Y): +Z conserva la pose perpendicular y -Z
# no se inclina porque su superficie está sobre la mesa. Con 80° la cara +Y
# llega a ser alcanzable, pero con incidencia muy rasante (se avisa).
DEFAULT_MAX_TILT_DEG: float = 80.0
TILT_FACES = frozenset({"+X", "-X", "+Y", "-Y"})
TILT_SEARCH_STEP_DEG: float = 5.0
TILT_COARSE_STEP_DEG: float = 15.0
TILT_AZIMUTH_COARSE_STEP_DEG: float = 30.0
TILT_WARNING_DEG: float = 45.0
MIN_JOINT_MARGIN_RAD: float = 0.02
MAX_JOINT_STEP_RAD: float = 0.5

_EPS = 1.0e-9
_ROTATION_EPS = 1.0e-6


def default_scan_paths_path() -> Path:
    """Ruta del YAML de barridos (`<config>/scan_paths.yaml`)."""
    return default_config_dir() / "scan_paths.yaml"


# ─── Utilidades vectoriales ──────────────────────────────────────────────


def _sub(a: Sequence[float], b: Sequence[float]) -> List[float]:
    return [float(a[i]) - float(b[i]) for i in range(3)]


def _add_scaled(a: Sequence[float], v: Sequence[float], k: float) -> List[float]:
    return [float(a[i]) + float(v[i]) * float(k) for i in range(3)]


def _scale(v: Sequence[float], k: float) -> List[float]:
    return [float(v[i]) * float(k) for i in range(3)]


def _norm(v: Sequence[float]) -> float:
    return math.sqrt(sum(float(c) * float(c) for c in v))


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(float(a[i]) * float(b[i]) for i in range(3))


def _cross(a: Sequence[float], b: Sequence[float]) -> List[float]:
    return [
        float(a[1]) * float(b[2]) - float(a[2]) * float(b[1]),
        float(a[2]) * float(b[0]) - float(a[0]) * float(b[2]),
        float(a[0]) * float(b[1]) - float(a[1]) * float(b[0]),
    ]


def _unit(v: Sequence[float]) -> List[float]:
    n = _norm(v)
    if n < _EPS:
        raise ValueError("no se puede normalizar un vector nulo.")
    return _scale(v, 1.0 / n)


def _rotation_from_columns(x: Sequence[float], y: Sequence[float], z: Sequence[float]) -> kdl.Rotation:
    """Construye la rotación con X/Y/Z como columnas (constructor de PyKDL)."""
    return kdl.Rotation(
        float(x[0]), float(y[0]), float(z[0]),
        float(x[1]), float(y[1]), float(z[1]),
        float(x[2]), float(y[2]), float(z[2]),
    )


def _rpy_degrees(rotation: kdl.Rotation) -> List[float]:
    rpy = rotation.GetRPY()
    return [float(math.degrees(v)) for v in (rpy[0], rpy[1], rpy[2])]


# ─── Orientación a partir del eje óptico real ────────────────────────────


def frame_looking_with_optical_axis(
    position: Sequence[float],
    pointing: Sequence[float],
    tangent: Sequence[float] | None = None,
) -> kdl.Frame:
    """Frame de tool0 tal que el **boresight del FOV** (el +X de
    `sensor_support_link`, que en tool0 coincide con +Z) apunta a `pointing`.

    Se construye el frame del FOV con X = dirección de apuntado y un
    referente estable para el resto, y luego se deshace el rpy de montaje:

        R_tool0 = R_FOV · R_montaje⁻¹

    Si se proporciona `tangent` (dirección de barrido en el plano de la cara),
    el giro alrededor del eje de apuntado se fija de forma coherente con el
    avance, evitando degeneraciones para cualquier cara plana. En caso contrario
    se mantiene el criterio histórico (compatibilidad hacia atrás).
    """
    x = _unit(pointing)
    if tangent is not None:
        t = _unit(tangent)
        # Proyectar el tangente sobre el plano perpendicular al boresight para
        # mantenerlo ortogonal y evitar degeneraciones.
        t = _unit(_sub(t, _scale(x, _dot(t, x))))
        # Y_sensor perpendicular a ambos (boresight y tangente proyectado)
        y = _unit(_cross(t, x))
        # Z_sensor perpendicular al avance, en el plano de la cara
        z = _cross(x, y)
    else:
        aux = [1.0, 0.0, 0.0] if abs(x[2]) > 0.99 else [0.0, 0.0, 1.0]
        z = _unit(_cross(aux, x))
        y = _cross(z, x)
    r_cam = _rotation_from_columns(x, y, z)
    r_mount = kdl.Rotation.RPY(*CAMERA_MOUNT_RPY_DEG)
    r_tool0 = r_cam * r_mount.Inverse()
    return kdl.Frame(r_tool0, kdl.Vector(*[float(v) for v in position]))


def camera_optical_axis(rotation: kdl.Rotation) -> List[float]:
    """Dirección a la que apunta el boresight del FOV (+X de
    `sensor_support_link`, que en tool0 es +Z), en base_link.

    A partir de la rotación de tool0 recompone el rpy de montaje. Es la
    comprobación directa de que la orientación derivada apunta a la cara.
    """
    r_mount = kdl.Rotation.RPY(*CAMERA_MOUNT_RPY_DEG)
    r_cam = rotation * r_mount
    axis = r_cam * kdl.Vector(1.0, 0.0, 0.0)
    return [float(axis[0]), float(axis[1]), float(axis[2])]


def sensor_mount_frame() -> kdl.Frame:
    """Pose FÍSICA de `sensor_support_link` respecto a `tool0` (mount real)."""
    return kdl.Frame(
        kdl.Rotation.RPY(*SENSOR_MOUNT_RPY),
        kdl.Vector(*SENSOR_MOUNT_XYZ),
    )


def frame_sensor_on_surface(
    sensor_position: Sequence[float],
    pointing: Sequence[float],
    tangent: Sequence[float] | None = None,
) -> Tuple[kdl.Frame, kdl.Frame]:
    """Devuelve ``(sensor_frame, tool0_frame)`` situando el ORIGEN DEL SENSOR.

    `sensor_frame` es la pose de `sensor_support_link` en base_link con su
    origen en `sensor_position` y su boresight (+X) apuntando a `pointing`.
    `tool0_frame` es la pose de la brida que lo materializa.

    El roll de tool0 lo fija la convención
    (`frame_looking_with_optical_axis`), que mantiene la línea roja del FOV
    perpendicular al avance. Desde esa orientación se RETROCEDE tool0 el
    vector de montaje FÍSICO para que sea el sensor —y no la brida— quien
    quede en la posición pedida. Ésa es la corrección de los ~169 mm.
    """
    tool0_dir = frame_looking_with_optical_axis(sensor_position, pointing, tangent=tangent)
    mount = sensor_mount_frame()
    tool0_pos = kdl.Vector(*[float(v) for v in sensor_position]) - tool0_dir.M * mount.p
    tool0 = kdl.Frame(tool0_dir.M, tool0_pos)
    return tool0 * mount, tool0


def sensor_optical_axis(rotation: kdl.Rotation) -> List[float]:
    """Boresight del sensor (+X de `sensor_support_link`) en base_link."""
    axis = rotation * kdl.Vector(1.0, 0.0, 0.0)
    return [float(axis[0]), float(axis[1]), float(axis[2])]


def tilted_optical_axis(
    face_normal: Sequence[float],
    tangent: Sequence[float],
    tilt_rad: float,
    azimuth_rad: float,
) -> List[float]:
    """Boresight inclinado `tilt_rad` respecto a la normal de la cara.

    `azimuth_rad` gira la inclinación dentro del plano de la cara: 0 sigue el
    sentido de barrido y π/2 su perpendicular. El resultado mira siempre hacia
    la superficie (componente -normal), sea cual sea el azimut.
    """
    n = _unit(face_normal)
    e1 = _unit(tangent)
    e1 = _unit(_sub(e1, _scale(n, _dot(e1, n))))
    e2 = _unit(_cross(n, e1))
    u = _add_scaled(_scale(e1, math.cos(azimuth_rad)), e2, math.sin(azimuth_rad))
    return _unit(_add_scaled(_scale(n, -math.cos(tilt_rad)), u, math.sin(tilt_rad)))


# ─── Fuente de verdad ────────────────────────────────────────────────────


@dataclass
class PartBox:
    """Cubo a inspeccionar, descrito en base_link."""

    center: List[float] = field(default_factory=lambda: list(DEFAULT_PART_CENTER))
    size: List[float] = field(default_factory=lambda: list(DEFAULT_PART_SIZE))

    def __post_init__(self) -> None:
        self.center = [float(v) for v in self.center]
        self.size = [float(v) for v in self.size]
        if len(self.center) != 3 or len(self.size) != 3:
            raise ValueError("center y size deben tener 3 componentes.")
        if any(s <= 0.0 for s in self.size):
            raise ValueError("los lados del cubo deben ser positivos.")

    @property
    def half_size(self) -> float:
        return max(self.size) / 2.0

    def half_extent_along(self, axis: Sequence[float]) -> float:
        """Semianchura REAL de la caja proyectada sobre `axis` (unitario).

        Es la mitad de la extensión de la pieza a lo largo de la dirección de
        barrido: define los BORDES reales de la cara sobre los que se aplican
        los márgenes, en lugar de repartir `line_length` a medias. Para un
        cubo y un eje alineado con sus lados coincide con `size/2`.
        """
        u = _unit(axis)
        return sum(abs(self.size[i] / 2.0 * u[i]) for i in range(3))

    def face_normal(self, face: str) -> List[float]:
        """Normal saliente de la cara indicada."""
        try:
            return list(FACE_NORMALS[face])
        except KeyError:
            raise ValueError(
                f"cara desconocida '{face}'; válidas: {', '.join(FACE_NORMALS)}."
            ) from None

    def face_center(self, face: str) -> List[float]:
        """Centro de la cara indicada, en base_link."""
        return _add_scaled(self.center, self.face_normal(face), self.half_size)

    def support_center(self, face: str) -> List[float]:
        """Centro del plano de apoyo desde el que se mide `scan_distance`.

        Para la cara superior (+Z) la pieza está apoyada sobre la mesa, así que
        la referencia es su cara inferior, que es el plano de contacto con la
        mesa. Así la cámara queda a `scan_distance` de la MESA y no de la cara
        superior de la pieza.

        Para el resto de caras el plano de apoyo es la propia cara: tomar la
        cara opuesta desplazaría el sensor el espesor de la pieza respecto a la
        superficie que se quiere recorrer.
        """
        if face == TABLE_BORNE_FACE:
            return self.face_center(OPPOSITE_FACE[face])
        return self.face_center(face)

    def signed_distance(self, position: Sequence[float], face: str) -> float:
        """Distancia con signo de `position` al plano de la cara."""
        return _dot(_sub(position, self.face_center(face)), self.face_normal(face))

    def signed_distance_to_support(self, position: Sequence[float], face: str) -> float:
        """Distancia con signo de `position` al plano de apoyo de `face`."""
        normal = self.face_normal(face)
        return _dot(_sub(position, self.support_center(face)), normal)


@dataclass
class ScanPathSpec:
    """Un barrido lineal: qué cara, y cómo es la línea sobre ella."""

    name: str
    face: str = "+Z"
    line_length: float = 0.08
    line_axis: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0])
    scan_margin_start: float = 0.0
    scan_margin_end: float = 0.0

    def __post_init__(self) -> None:
        self.name = str(self.name)
        self.face = str(self.face)
        self.line_length = float(self.line_length)
        self.line_axis = [float(v) for v in self.line_axis]
        self.scan_margin_start = float(self.scan_margin_start)
        self.scan_margin_end = float(self.scan_margin_end)
        if not self.name:
            raise ValueError("el barrido necesita un nombre.")
        if len(self.line_axis) != 3:
            raise ValueError("line_axis debe tener 3 componentes.")
        if _norm(self.line_axis) < _EPS:
            raise ValueError("line_axis no puede ser nulo.")
        if self.line_length <= 0.0:
            raise ValueError("line_length debe ser positivo.")
        if self.scan_margin_start < 0.0:
            raise ValueError("scan_margin_start no puede ser negativo.")
        if self.scan_margin_end < 0.0:
            raise ValueError("scan_margin_end no puede ser negativo.")
        if self.face not in FACE_NORMALS:
            raise ValueError(
                f"cara desconocida '{self.face}'; válidas: {', '.join(FACE_NORMALS)}."
            )


@dataclass
class ScanProfile:
    """Parcela global del barrido: pieza y distancia de escaneo."""

    part: PartBox = field(default_factory=PartBox)
    scan_distance: float = DEFAULT_SCAN_DISTANCE
    max_tilt_deg: float = DEFAULT_MAX_TILT_DEG

    def __post_init__(self) -> None:
        if not isinstance(self.part, PartBox):
            self.part = PartBox(center=list(self.part.center), size=list(self.part.size))
        self.scan_distance = float(self.scan_distance)
        self.max_tilt_deg = float(self.max_tilt_deg)
        if self.scan_distance <= 0.0:
            raise ValueError("scan_distance debe ser positivo.")
        if not 0.0 <= self.max_tilt_deg < 90.0:
            raise ValueError("max_tilt_deg debe estar en [0, 90).")

    def tangent_axis(self, spec: ScanPathSpec) -> List[float]:
        """Sentido de la línea, unitario y contenido en el plano de la cara.

        Se proyecta el `line_axis` declarado sobre el plano tangente a la
        cara, de forma que un eje ligeramente torcido (o parallelism
        accidental con la normal) no rompe la geometría ni la distancia.
        """
        normal = self.part.face_normal(spec.face)
        proyectado = _sub(spec.line_axis, _scale(normal, _dot(spec.line_axis, normal)))
        if _norm(proyectado) < _EPS:
            raise ValueError(
                f"line_axis es paralelo a la normal de la cara {spec.face} "
                "y no define ninguna línea de barrido."
            )
        return _unit(proyectado)

    def scan_span(self, spec: ScanPathSpec) -> List[List[float]]:
        """Bordes REALES de la zona inspeccionada: ``[entrada, salida]``.

        Se calculan sobre el plano de apoyo, sin el desplazamiento
        perpendicular: es la huella del barrido sobre la pieza.

            entrada = borde_entrada  − eje·margen_start
            salida  = borde_opuesto  + eje·margen_end

        `borde_entrada`/`borde_opuesto` son los extremos REALES de la pieza
        (extensión de `PartBox` proyectada sobre el avance), no `line_length/2`:
        así el tramo inspeccionado SIEMPRE contiene a la pieza entera y cada
        margen cae FUERA de su borde correspondiente. `line_length` es la
        longitud INSPECIONADA (informativa; por defecto el lado del cubo) y no
        desplaza los extremos.

        `eje` es el sentido de AVANCE del barrido, P1 -> P2, tal cual sale de
        `tangent_axis(spec)`. No se presupone ninguna dirección: el borde de
        entrada es el de arranque (−eje) y el margen start lo aleja hacia
        −eje (hacia fuera); el de salida (+eje) lo aleja `margen_end` hacia
        +eje. Con el sentido por defecto de +Z (`line_axis = [0,1,0]`) el
        arranque cae en −Y y la llegada en +Y; con `[0,-1,0]` al revés.
        """
        center = self.part.support_center(spec.face)
        axis = self.tangent_axis(spec)
        half_extent = self.part.half_extent_along(axis)
        entry = _add_scaled(center, axis, -(half_extent + spec.scan_margin_start))
        exit_ = _add_scaled(center, axis, half_extent + spec.scan_margin_end)
        return [entry, exit_]

    def target_positions(self, spec: ScanPathSpec) -> List[List[float]]:
        """Posiciones del SENSOR en P1 y P2, en base_link.

            P1 = entrada + normal·d
            P2 = salida  + normal·d

        `entrada`/`salida` son los bordes de la zona inspeccionada que devuelve
        `scan_span` (borde real de la pieza, extendido hacia fuera por el
        margen correspondiente). `centro_apoyo` es el centro del plano sobre el
        que se apoya la pieza, así que `scan_distance` se aplica desde ahí: el
        SENSOR queda a esa distancia de la MESA (cara +Z) o de la propia cara
        (resto).

        Los márgenes son ASIMÉTRICOS e independientes y se miden siempre a lo
        largo de `eje` (el avance P1 -> P2), nunca sobre una coordenada suelta:
        la línea sigue siendo perpendicular a la cara y `scan_distance`
        constante en todo el recorrido. Margen 0 deja ese extremo exactamente
        en el borde real de la pieza.
        """
        normal = self.part.face_normal(spec.face)
        entry, exit_ = self.scan_span(spec)
        return [
            _add_scaled(entry, normal, self.scan_distance),
            _add_scaled(exit_, normal, self.scan_distance),
        ]


# ─── Datos derivados ─────────────────────────────────────────────────────


@dataclass
class ScanPoint:
    """Un punto DERIVADO: pose cartesiana + q de la IK real + validación."""

    role: str
    position_xyz: List[float]
    rpy_degrees: List[float]
    quaternion_xyzw: List[float]
    optical_axis: List[float]
    q: List[float] = field(default_factory=lambda: [0.0] * 6)
    solved: bool = False
    inside_limits: bool = False
    offending_joints: List[int] = field(default_factory=list)
    distance_measured: float = math.inf
    distance_error: float = math.inf
    pos_error: float = math.inf
    rot_error: float = math.inf

    @property
    def valid(self) -> bool:
        return self.solved and self.inside_limits

    def offending_joint_names(self) -> List[str]:
        return [f"joint_{i}" for i in self.offending_joints]


@dataclass
class ScanSolution:
    """Resultado de resolver un `ScanPathSpec` contra un `ScanProfile`."""

    spec: ScanPathSpec
    profile: ScanProfile
    points: List[ScanPoint] = field(default_factory=list)
    tilt_deg: float = 0.0
    tilt_azimuth_deg: float = 0.0
    warnings: List[str] = field(default_factory=list)

    def __iter__(self):
        return iter(self.points)

    def __len__(self) -> int:
        return len(self.points)

    def point(self, role: str) -> Optional[ScanPoint]:
        for p in self.points:
            if p.role == role:
                return p
        return None

    @property
    def valid(self) -> bool:
        """True si los dos puntos resuelven y respetan los límites."""
        return len(self.points) == len(ROLES) and all(p.valid for p in self.points)

    @property
    def offending_joints(self) -> List[int]:
        """Articulaciones que impiden el barrido, sin repetir y ordenadas."""
        out: List[int] = []
        for p in self.points:
            for j in p.offending_joints:
                if j not in out:
                    out.append(j)
        return sorted(out)

    @property
    def scan_length(self) -> float:
        p1, p2 = self.point("P1"), self.point("P2")
        if p1 is None or p2 is None:
            return 0.0
        return _norm(_sub(p2.position_xyz, p1.position_xyz))


def load_kinematics():
    """Instancia la cinemática real (import perezoso, como en gui.py)."""
    from staubli_trajectory.tx2_60l_ik import Tx260lKinematics

    return Tx260lKinematics()


def _build_scan_points(
    profile: ScanProfile,
    spec: ScanPathSpec,
    kin,
    positions: Sequence[Sequence[float]],
    optical_dir: Sequence[float],
    tangent: Sequence[float],
    mount: kdl.Frame,
    normal: Sequence[float],
    require_valid: bool = False,
) -> Optional[List[ScanPoint]]:
    """Resuelve P1/P2 para una orientación común y mide la distancia axial.

    Con `require_valid=True` se abandona en cuanto un punto no es válido: es la
    ruta de búsqueda, donde sólo interesan los candidatos completos.
    """
    points: List[ScanPoint] = []
    q_prev: Optional[List[float]] = None
    for role, position in zip(ROLES, positions):
        sensor_frame, tool0_frame = frame_sensor_on_surface(
            position, optical_dir, tangent=tangent
        )
        result = kin.inverse_kinematics(tool0_frame, q_prev=q_prev)

        if result.solved:
            inside, per_joint = kin.check_joint_limits(result.q)
            offending = [i + 1 for i, ok in enumerate(per_joint) if not ok]
            sensor_world = kin.forward_kinematics(result.q) * mount
            measured = sensor_world.p
            measured_pos = [float(measured[0]), float(measured[1]), float(measured[2])]
            signed = profile.part.signed_distance_to_support(measured_pos, spec.face)
            # Distancia AXIAL a lo largo del boresight hasta el plano de apoyo.
            # Con el sensor apuntando a la cara, facing = -cos(tilt) < 0.
            facing = _dot(sensor_optical_axis(sensor_world.M), normal)
            distance = abs(signed / facing) if facing < -_EPS else math.inf
            # Filtro geométrico conservador: mesa/pieza vs cuerpo sensor
            env_col, env_reason = check_sensor_environment_collision(sensor_world, profile.part)
            if env_col:
                inside = False
                if env_reason and env_reason not in offending:
                    # marcar como no válido por colisión ambiental (se registra vía inside_limits False)
                    pass
        else:
            inside, offending = False, []
            distance = math.inf

        sp = ScanPoint(
            role=role,
            position_xyz=[float(v) for v in position],
            rpy_degrees=_rpy_degrees(sensor_frame.M),
            quaternion_xyzw=[float(v) for v in sensor_frame.M.GetQuaternion()],
            optical_axis=sensor_optical_axis(sensor_frame.M),
            q=[float(v) for v in result.q],
            solved=bool(result.solved),
            inside_limits=bool(inside),
            offending_joints=offending,
            distance_measured=distance,
            distance_error=(
                distance - profile.scan_distance if math.isfinite(distance) else math.inf
            ),
            pos_error=float(result.pos_error),
            rot_error=float(result.rot_error),
        )
        # Adjuntar motivo si hay colisión ambiental
        if result.solved and 'env_col' in locals() and env_col:
            sp.inside_limits = False
            sp.solved = True  # keep solved but mark invalid
        points.append(sp)
        if require_valid and not points[-1].valid:
            return None
        if result.solved and points[-1].valid:
            q_prev = result.q
        elif result.solved and not points[-1].valid:
            # no encadenar con pose inválida por colisión
            q_prev = q_prev
    # Validación preliminar de puntos intermedios (muestreo) - trayectoria real puede diferir (interpolación articular)
    if len(points) == 2 and all(p.valid for p in points):
        try:
            q1, q2 = points[0].q, points[1].q
            for alpha in (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9):
                qmid = [q1[i] + alpha*(q2[i]-q1[i]) for i in range(6)]
                Tmid = kin.forward_kinematics(qmid)
                Smid = Tmid * mount
                ec, er = check_sensor_environment_collision(Smid, profile.part)
                if ec:
                    return None
        except Exception:
            pass
    return points


def _joint_margin(q: Sequence[float], soft_limits) -> float:
    return min(
        min(float(q[i]) - soft_limits[i][0], soft_limits[i][1] - float(q[i]))
        for i in range(6)
    )


def _search_tilted_scan(
    profile: ScanProfile,
    spec: ScanPathSpec,
    kin,
    tangent: Sequence[float],
    normal: Sequence[float],
    mount: kdl.Frame,
) -> Optional[Tuple[List[ScanPoint], float, float]]:
    """Busca una inclinación que haga válido el barrido lateral.

    Se barre primero la inclinación en pasos gruesos. En cada una se prueban
    azimuts hasta dar con el primer candidato que resuelve P1 y P2 dentro de
    límites, con margen a los límites blandos y sin un salto articular grande.
    Después se afina hacia abajo en pasos finos para no quedarse con una
    inclinación mayor de la necesaria.
    """
    from staubli_trajectory.tx2_60l_ik import JOINT_SOFT_LIMITS_RAD

    if profile.max_tilt_deg <= 0.0 or spec.face not in TILT_FACES:
        return None

    face_points = profile.scan_span(spec)
    distance = profile.scan_distance

    def evaluate(tilt_deg: float, azimuth_deg: float):
        optical_dir = tilted_optical_axis(
            normal,
            tangent,
            math.radians(tilt_deg),
            math.radians(azimuth_deg),
        )
        positions = [
            _sub(point, _scale(optical_dir, distance)) for point in face_points
        ]
        points = _build_scan_points(
            profile,
            spec,
            kin,
            positions,
            optical_dir,
            tangent,
            mount,
            normal,
            require_valid=True,
        )
        if points is None:
            return None
        margin = min(_joint_margin(point.q, JOINT_SOFT_LIMITS_RAD) for point in points)
        step = max(abs(points[1].q[i] - points[0].q[i]) for i in range(6))
        if margin < MIN_JOINT_MARGIN_RAD or step > MAX_JOINT_STEP_RAD:
            return None
        return points

    def first_at(tilt_deg: float):
        for azimuth_deg in range(0, 360, int(TILT_AZIMUTH_COARSE_STEP_DEG)):
            points = evaluate(tilt_deg, azimuth_deg)
            if points is not None:
                return points, tilt_deg, float(azimuth_deg)
        return None

    coarse = TILT_COARSE_STEP_DEG
    max_tilt = profile.max_tilt_deg
    if max_tilt <= 3.0 * coarse:
        # Rango corto: no merece la pena una malla gruesa que podría saltarse
        # la única inclinación válida.
        grid = []
        tilt = TILT_SEARCH_STEP_DEG
        while tilt < max_tilt - _EPS:
            grid.append(tilt)
            tilt += TILT_SEARCH_STEP_DEG
        grid.append(max_tilt)
    else:
        grid = []
        tilt = coarse
        while tilt < max_tilt - _EPS:
            grid.append(tilt)
            tilt += coarse
        grid.append(max_tilt)

    for tilt in grid:
        found = first_at(tilt)
        if found is None:
            continue
        # Afinar hacia abajo mientras la inclinación menor siga funcionando.
        lower = found[1] - TILT_SEARCH_STEP_DEG
        while lower > 0.0:
            candidate = first_at(lower)
            if candidate is None:
                break
            found = candidate
            lower = candidate[1] - TILT_SEARCH_STEP_DEG
        return found

    return None


def resolve_scan(
    profile: ScanProfile,
    spec: ScanPathSpec,
    kinematics=None,
) -> ScanSolution:
    """Deriva P1/P2 de la geometría y los valida con la IK real.

    Primero se prueba la orientación ideal (boresight perpendicular a la cara).
    Si P1/P2 no son válidos en una cara lateral, se busca una inclinación
    creciente hasta `profile.max_tilt_deg`; +Z y -Z conservan la pose ideal.

    P1/P2 son posiciones del SENSOR: la IK se resuelve sobre la pose de tool0
    que, aplicándole el montaje físico, deja `sensor_support_link` en ese
    punto. `position_xyz` / `rpy_degrees` describen el SENSOR y
    `distance_measured` es la distancia AXIAL desde el ORIGEN DEL SENSOR (vía
    FK) hasta el plano de apoyo, a lo largo del boresight.

    La IK se encadena (`q_prev`) para que el solver elija la rama continua
    entre puntos y no dé un salto articular en mitad del barrido.
    """
    kin = kinematics if kinematics is not None else load_kinematics()
    normal = profile.part.face_normal(spec.face)
    tangent = profile.tangent_axis(spec)
    mount = sensor_mount_frame()

    ideal_positions = profile.target_positions(spec)
    points = _build_scan_points(
        profile,
        spec,
        kin,
        ideal_positions,
        _scale(normal, -1.0),
        tangent,
        mount,
        normal,
    )
    solution = ScanSolution(spec=spec, profile=profile, points=points)
    if solution.valid:
        return solution

    tilted = _search_tilted_scan(profile, spec, kin, tangent, normal, mount)
    if tilted is None:
        return solution

    tilted_points, tilt_deg, azimuth_deg = tilted
    solution = ScanSolution(
        spec=spec,
        profile=profile,
        points=tilted_points,
        tilt_deg=float(tilt_deg),
        tilt_azimuth_deg=float(azimuth_deg),
    )
    if tilt_deg > TILT_WARNING_DEG:
        solution.warnings.append(
            f"inclinación de {tilt_deg:.1f}° (>{TILT_WARNING_DEG:.0f}°): "
            "incidencia oblicua y cobertura reducida."
        )
    return solution


# ─── Serialización (tolerante) ───────────────────────────────────────────


def _to_float_list(value, length: int) -> Optional[List[float]]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        return None
    try:
        return [float(v) for v in value]
    except (TypeError, ValueError):
        return None


def _partbox_from_dict(data) -> Optional[PartBox]:
    if not isinstance(data, dict):
        return None
    center = _to_float_list(data.get("center"), 3)
    size = _to_float_list(data.get("size"), 3)
    if center is None or size is None:
        return None
    try:
        return PartBox(center=center, size=size)
    except ValueError:
        return None


def _scanpathspec_from_dict(data) -> Optional[ScanPathSpec]:
    if not isinstance(data, dict):
        return None
    name = data.get("name")
    face = data.get("face")
    if not name or not face:
        return None
    line_length = data.get("line_length", 0.08)
    line_axis = _to_float_list(data.get("line_axis", [1.0, 0.0, 0.0]), 3)
    if line_axis is None:
        return None
    try:
        return ScanPathSpec(
            name=str(name),
            face=str(face),
            line_length=float(line_length),
            line_axis=line_axis,
            scan_margin_start=float(data.get("scan_margin_start", 0.0)),
            scan_margin_end=float(data.get("scan_margin_end", 0.0)),
        )
    except (TypeError, ValueError):
        return None


def _partbox_to_dict(part: PartBox) -> Dict[str, object]:
    return {"center": [float(v) for v in part.center], "size": [float(v) for v in part.size]}


def _scanpathspec_to_dict(spec: ScanPathSpec) -> Dict[str, object]:
    return {
        "name": spec.name,
        "face": spec.face,
        "line_length": float(spec.line_length),
        "line_axis": [float(v) for v in spec.line_axis],
        "scan_margin_start": float(spec.scan_margin_start),
        "scan_margin_end": float(spec.scan_margin_end),
    }


# ─── Almacén ─────────────────────────────────────────────────────────────


class ScanPathStore:
    """Carga/guardado de `scan_paths.yaml` (escritura atómica).

    Sólo persiste la fuente de verdad (`part`, `scan_distance` y la lista de
    especificaciones). P1/P2 no se tocan: se derivan con `resolve()` en cada
    lectura, de modo que el fichero nunca puede quedar desincronizado de la
    geometría que lo describe.
    """

    VERSION = 3

    # Valor de `scan_distance` de la v1 que quedó obsoleto al fijar la
    # distancia de trabajo en 0.325 m. Se migra al cargar (ver `load`).
    LEGACY_SCAN_DISTANCE = 0.50

    def __init__(self, path: Optional[Path] = None) -> None:
        self.path = Path(path) if path is not None else default_scan_paths_path()
        self.profile = ScanProfile()
        self._specs: Dict[str, ScanPathSpec] = {}
        self.errors: List[str] = []
        self.load()

    # ── carga ──
    def load(self) -> None:
        """Recarga desde disco; entradas inválidas se omiten y se anotan."""
        self._specs = {}
        self.profile = ScanProfile()
        self.errors = []
        if not self.path.is_file():
            return
        try:
            raw = yaml.safe_load(self.path.read_text(encoding="utf-8"))
        except Exception as exc:  # noqa: BLE001 - reporte tolerante
            self.errors.append(f"no se pudo leer {self.path}: {exc}")
            return
        if not isinstance(raw, dict):
            self.errors.append(f"formato inesperado en {self.path} (se omite).")
            return

        part = _partbox_from_dict(raw.get("part"))
        if part is None:
            self.errors.append("'part' ausente o inválido (se usan valores por defecto).")
        else:
            self.profile.part = part

        raw_version = raw.get("version", 1)
        try:
            raw_version = int(raw_version)
        except (TypeError, ValueError):
            raw_version = 1

        distance = raw.get("scan_distance", DEFAULT_SCAN_DISTANCE)
        # Migración v1 -> v2: la v1 traía 0.50 m por defecto, una distancia que
        # se descartó al sintonizar el sensor. No es un valor que el usuario
        # eligiera a propósito, así que se traduce al nuevo defecto.
        if raw_version < 2 and isinstance(distance, (int, float)) and math.isclose(
            float(distance), self.LEGACY_SCAN_DISTANCE, abs_tol=1e-9
        ):
            distance = DEFAULT_SCAN_DISTANCE
        max_tilt = raw.get("max_tilt_deg", DEFAULT_MAX_TILT_DEG)
        try:
            distance = float(distance)
        except (TypeError, ValueError):
            self.errors.append(
                "scan_distance inválido; se usa el valor por defecto."
            )
            distance = DEFAULT_SCAN_DISTANCE
        try:
            max_tilt = float(max_tilt)
        except (TypeError, ValueError):
            self.errors.append(
                "max_tilt_deg inválido; se usa el valor por defecto."
            )
            max_tilt = DEFAULT_MAX_TILT_DEG
        if distance <= 0.0:
            self.errors.append(
                "scan_distance debe ser positivo; se usa el valor por defecto."
            )
            distance = DEFAULT_SCAN_DISTANCE
        if not 0.0 <= max_tilt < 90.0:
            self.errors.append(
                "max_tilt_deg debe estar en [0, 90); se usa el valor por defecto."
            )
            max_tilt = DEFAULT_MAX_TILT_DEG
        self.profile = ScanProfile(
            part=self.profile.part,
            scan_distance=distance,
            max_tilt_deg=max_tilt,
        )

        for i, entry in enumerate(raw.get("scan_paths", []) or []):
            spec = _scanpathspec_from_dict(entry)
            if spec is None:
                self.errors.append(f"barrido {i + 1} inválido (omitido).")
                continue
            if spec.name in self._specs:
                self.errors.append(
                    f"nombre duplicado '{spec.name}' (se mantiene el primero)."
                )
                continue
            self._specs[spec.name] = spec

    # ── guardado ──
    def save(self) -> None:
        """Guarda el fichero en una escritura atómica (temp + os.replace)."""
        self.path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "version": self.VERSION,
            "part": _partbox_to_dict(self.profile.part),
            "scan_distance": float(self.profile.scan_distance),
            "max_tilt_deg": float(self.profile.max_tilt_deg),
            "scan_paths": [_scanpathspec_to_dict(s) for s in self.all()],
        }
        text = yaml.safe_dump(data, sort_keys=False, allow_unicode=True)
        tmp = self.path.with_name(self.path.name + ".tmp")
        tmp.write_text(text, encoding="utf-8")
        os.replace(tmp, self.path)

    # ── acceso ──
    def names(self) -> List[str]:
        return list(self._specs)

    def all(self) -> List[ScanPathSpec]:
        return list(self._specs.values())

    def get(self, name: str) -> Optional[ScanPathSpec]:
        return self._specs.get(name)

    def has(self, name: str) -> bool:
        return name in self._specs

    def add(self, spec: ScanPathSpec) -> None:
        if spec.name in self._specs:
            raise ValueError(f"ya existe un barrido llamado '{spec.name}'.")
        self._specs[spec.name] = spec

    def update(self, spec: ScanPathSpec, previous_name: Optional[str] = None) -> None:
        """Sustituye un barrido; soporta el renombrado si 'previous_name' difiere."""
        if previous_name and previous_name != spec.name:
            if previous_name not in self._specs:
                raise ValueError(f"el barrido '{previous_name}' no existe.")
            self._specs.pop(previous_name)
        if spec.name in self._specs and spec.name != previous_name:
            raise ValueError(f"ya existe un barrido llamado '{spec.name}'.")
        self._specs[spec.name] = spec

    def remove(self, name: str) -> None:
        if name not in self._specs:
            raise KeyError(f"el barrido '{name}' no existe.")
        self._specs.pop(name)

    # ── parámetros globales ──
    def set_scan_distance(self, value: float) -> None:
        self.profile.scan_distance = float(value)

    def set_max_tilt_deg(self, value: float) -> None:
        value = float(value)
        if not 0.0 <= value < 90.0:
            raise ValueError("max_tilt_deg debe estar en [0, 90).")
        self.profile.max_tilt_deg = value

    def set_part(self, part: PartBox) -> None:
        self.profile.part = part

    # ── derivación ──
    def resolve(self, name: str, kinematics=None) -> Optional[ScanSolution]:
        """Deriva P1/P2 del barrido indicado con la IK real."""
        spec = self._specs.get(name)
        if spec is None:
            return None
        return resolve_scan(self.profile, spec, kinematics)

    def resolve_all(self, kinematics=None) -> List[ScanSolution]:
        """Deriva todos los barridos, en el orden de definición."""
        kin = kinematics if kinematics is not None else load_kinematics()
        return [resolve_scan(self.profile, spec, kin) for spec in self.all()]
