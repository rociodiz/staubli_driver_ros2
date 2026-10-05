"""Geometría de barrido y persistencia de los puntos P1/P2 sobre un cubo.

Un `ScanPathSpec` describe UN barrido lineal sobre UNA de las seis caras de un
cubo. La fuente de verdad es geométrica y deliberadamente mínima:

    part:           centro y lado del cubo, en base_link
    scan_distance:  distancia del sensor al PLANO DE APOYO del cubo, en metros
    scan_paths[]:   nombre, cara, longitud y sentido de la línea

`scan_distance` se mide desde el plano sobre el que se apoya el cubo —la mesa
en la celda de inspección—, NO desde su cara superior. La pieza sobresale por
encima de ese plano y es precisamente el volumen que la cámara debe capturar,
así que tomar la cara superior como referencia dejaría la cámara demasiado
baja para barrerla.

Los puntos P1/P2 y su validación NO se guardan: son DERIVADOS. Se recalculan
con `ScanPathStore.resolve()` a partir de la geometría y de la IK real del
TX2-60L, de forma que cambiar `scan_distance` no deja residuos obsoletos en
disco y dos ejecuciones del mismo perfil dan exactamente el mismo barrido.

Convención de orientación
-------------------------
La cámara es eye-in-hand (`staubli_camera.xacro`) y su eje óptico real es el
**+X de camera_link**, no el -Z que afirma el comentario de ese xacro: en
Gazebo las cámaras miran por +X. Los tres puntos se orientan de modo que ese
+X apunte a la superficie, compensando el rpy de montaje con

    R_tool0 = R_camera · R_montaje⁻¹

`tool0` y `camera_link` están ambos fijos a `link_6` en `xyz="0 0 0.07"`, pero
sólo la cámara lleva rpy, así que la rotación de tool0 respecto a link_6 es la
identidad y `R_base_tool0 = R_base_cam · R_montaje⁻¹` directamente.

Consecuencia medida de esta convención: la cámara mira perpendicular a la cara
mientras el brazo extiende el sensor casi radialmente, así que la muñeca
(`joint_5`, ±120°) queda muy exigida en algunas orientaciones. Con el cubo de
la escena de inspección y línea de 8 cm, midiendo `scan_distance` desde el
plano de apoyo: +Z llega hasta d≈0.50 m (la cota de trabajo, la distancia a la
mesa), +X y -Y hasta 0.35 m, -X hasta 0.25 m y a partir de 0.30 m se sale de
`joint_5`; -Z se pasa de las articulaciones 2/3/5 y +Y no resuelve a ninguna
distancia. `resolve()` lo reporta en `ScanPoint.offending_joints` y
`ScanPoint.solved` en vez de ocultarlo, para que quien use esto distinga "fuera
de límites" de "no converge".

Nota sobre determinismo: `Tx260lKinematics` es multi-start y sortea sus
semillas de un `random.Random` cuyo estado avanza en cada llamada, así que
"resuelve fuera de límites" y "no converge" pueden alternarse para un mismo
objetivo según cuántas llamadas de IK se hayan hecho antes. Con una instancia
nueva por barrido el veredicto es reproducible.

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

# ─── Montaje de la cámara (espejo de staubli_camera.xacro) ───────────────
#
# NO se lee el xacro en runtime: son valores fijos de la descripción del
# robot, y releer un URDF por cada barrido.enums I/O innecesario. Se
# replican aquí para que la geometría sea testeable sin lanzar Gazebo.
#
# OJO con las unidades: <origin rpy="..."> del URDF se expresa en RADIANES,
# así que estos números se pasan a `kdl.Rotation.RPY` tal cual, sin
# convertir. Convertirlos desde grados (como se hacía antes) modelaba un
# montaje de 8.76° en lugar del real de 199.64°, y desviaba el eje óptico
# 163.36°: la cámara apuntaba a la pared en vez de a la cara +Z.
# El sufijo _DEG del nombre es histórico y engañoso; los valores son radianes.

CAMERA_MOUNT_RPY_DEG: Tuple[float, float, float] = (2.9391, -1.3969, -2.9714)
TOOL0_MOUNT_RPY_DEG: Tuple[float, float, float] = (0.0, 0.0, 0.0)

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
# la geometría validada en lugar de un cubo en el origen.
DEFAULT_PART_CENTER: List[float] = [0.0, 0.66, 0.24]
DEFAULT_PART_SIZE: List[float] = [0.08, 0.08, 0.08]

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
    position: Sequence[float], pointing: Sequence[float]
) -> kdl.Frame:
    """Frame de tool0 tal que el **+X de camera_link** apunta a `pointing`.

    Se construye el frame de la cámara con X = dirección de apuntado y un
   Referente estable para el resto (mismo criterio que
    `Tx260lKinematics.orientation_frame_looking_at`, pero sobre el eje óptico
    real en lugar del +Z de tool0), y luego se deshace el rpy de montaje:

        R_tool0 = R_camera · R_montaje⁻¹

    El giro alrededor del eje de apuntado queda fijado de forma determinista:
    el "arriba" de la cámara se alinea con +Z del mundo salvo que el apuntado
    sea casi vertical, en cuyo caso se usa +X como referente para que los
    ejes no se degeneren.
    """
    x = _unit(pointing)
    aux = [1.0, 0.0, 0.0] if abs(x[2]) > 0.99 else [0.0, 0.0, 1.0]
    z = _unit(_cross(aux, x))
    y = _cross(z, x)
    r_cam = _rotation_from_columns(x, y, z)
    r_mount = kdl.Rotation.RPY(*CAMERA_MOUNT_RPY_DEG)
    r_tool0 = r_cam * r_mount.Inverse()
    return kdl.Frame(r_tool0, kdl.Vector(*[float(v) for v in position]))


def camera_optical_axis(rotation: kdl.Rotation) -> List[float]:
    """Dirección a la que apunta el +X de camera_link, en base_link.

    A partir de la rotación de tool0 recompone el rpy de montaje. Es la
    comprobación directa de que la orientación derivada apunta a la cara.
    """
    r_mount = kdl.Rotation.RPY(*CAMERA_MOUNT_RPY_DEG)
    r_cam = rotation * r_mount
    axis = r_cam * kdl.Vector(1.0, 0.0, 0.0)
    return [float(axis[0]), float(axis[1]), float(axis[2])]


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
    scan_distance: float = 0.50

    def __post_init__(self) -> None:
        if not isinstance(self.part, PartBox):
            self.part = PartBox(center=list(self.part.center), size=list(self.part.size))
        self.scan_distance = float(self.scan_distance)
        if self.scan_distance <= 0.0:
            raise ValueError("scan_distance debe ser positivo.")

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

    def target_positions(self, spec: ScanPathSpec) -> List[List[float]]:
        """Posiciones de sensor de P1 y P2, en base_link.

            P1 = centro_apoyo − eje·(L/2 + margen_start) + normal·d
            P2 = centro_apoyo + eje·(L/2 + margen_end)  + normal·d

        `centro_apoyo` es el centro de la cara opuesta a la que se barre, es
        decir el plano sobre el que se apoya la pieza. `scan_distance` se
        aplica desde ahí, de modo que la cámara queda a esa distancia de la
        MESA y no de la cara superior de la pieza.

        `eje` es el sentido de AVANCE del barrido, P1 -> P2. Con la
        configuración de la celda (`line_axis = [0,-1,0]`) el barrido va de +Y
        a -Y: P1 es el extremo de arranque en +Y y P2 el de llegada en -Y.

        Los márgenes son ASIMÉTRICOS e independientes:

          - `scan_margin_start` extiende P1 hacia -eje, es decir hacia el lado
            de arranque. Con eje = [0,-1,0] eso es +Y, el lado ANTERIOR a la
            pieza: el barrido empieza antes de entrar en ella.
          - `scan_margin_end` extiende P2 hacia +eje, es decir -Y, el lado
            POSTERIOR: el barrido termina después de salir.

        Margen 0 en cualquiera de los dos deja ese extremo exactamente en el
        borde de la pieza, que es el comportamiento original.

        El desplazamiento va siempre a lo largo de `eje`, nunca sobre una
        coordenada suelta: la línea sigue siendo perpendicular a la cara y
        `scan_distance` constante en todo el recorrido.
        """
        normal = self.part.face_normal(spec.face)
        center = self.part.support_center(spec.face)
        axis = self.tangent_axis(spec)
        half_len = spec.line_length / 2.0
        p1 = _add_scaled(
            _add_scaled(center, axis, -(half_len + spec.scan_margin_start)),
            normal,
            self.scan_distance,
        )
        p2 = _add_scaled(
            _add_scaled(center, axis, half_len + spec.scan_margin_end),
            normal,
            self.scan_distance,
        )
        return [p1, p2]


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


def resolve_scan(
    profile: ScanProfile,
    spec: ScanPathSpec,
    kinematics=None,
) -> ScanSolution:
    """Deriva P1/P2 de la geometría y los valida con la IK real.

    Los dos puntos comparten orientación (la cámara mira a la cara), lo que
    garantiza por construcción que la distancia perpendicular se mantiene
    constante en todo el segmento de escaneo P1 → P2.

    La IK se encadena (`q_prev`) para que el solver elija la rama continua
    entre puntos y no dé un salto articular en mitad del barrido.
    """
    kin = kinematics if kinematics is not None else load_kinematics()
    positions = profile.target_positions(spec)
    normal = profile.part.face_normal(spec.face)
    optical_dir = _scale(normal, -1.0)

    solution = ScanSolution(spec=spec, profile=profile)
    q_prev: Optional[List[float]] = None
    for role, position in zip(ROLES, positions):
        frame = frame_looking_with_optical_axis(position, optical_dir)
        result = kin.inverse_kinematics(frame, q_prev=q_prev)

        if result.solved:
            inside, per_joint = kin.check_joint_limits(result.q)
            offending = [i + 1 for i, ok in enumerate(per_joint) if not ok]
            measured = kin.forward_kinematics(result.q).p
            measured_pos = [float(measured[0]), float(measured[1]), float(measured[2])]
            distance = abs(
                profile.part.signed_distance_to_support(measured_pos, spec.face)
            )
        else:
            inside, offending = False, []
            distance = math.inf

        solution.points.append(
            ScanPoint(
                role=role,
                position_xyz=[float(v) for v in position],
                rpy_degrees=_rpy_degrees(frame.M),
                quaternion_xyzw=[float(v) for v in frame.M.GetQuaternion()],
                optical_axis=camera_optical_axis(frame.M),
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
        )
        if result.solved:
            q_prev = result.q
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

    VERSION = 1

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

        distance = raw.get("scan_distance", 0.50)
        try:
            self.profile = ScanProfile(
                part=self.profile.part,
                scan_distance=float(distance),
            )
        except (TypeError, ValueError) as exc:
            self.errors.append(f"scan_distance inválido ({exc}); se usan valores por defecto.")
            self.profile = ScanProfile(part=self.profile.part)

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
