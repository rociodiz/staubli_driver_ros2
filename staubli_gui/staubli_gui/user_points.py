"""Modelo y persistencia de los puntos de inspección definidos por el usuario.

Cada punto representa una pose de tool0 respecto a base_link definida a mano
desde la GUI:

    - name              : identificador único
    - position_xyz      : [X, Y, Z]  (metros, frame base_link)
    - rpy_degrees       : [RX, RY, RZ] (grados, convención RPY estándar
                          R = Rz(yaw)·Ry(pitch)·Rx(roll), la misma que usa
                          PyKDL/ROS)
    - quaternion_xyzw   : [x, y, z, w] calculado a partir del RPY (convención
                          del proyecto: KDL/ROS, w último)
    - q                 : [q1..q6] resuelto por la IK de Tx260lKinematics
    - solved            : la IK convergió dentro de tolerancia
    - inside_limits     : la configuración respeta los límites articulares

Solo se guardan puntos con solved == True y inside_limits == True.

Persistencia (independiente de config/points.yaml del paquete):

    $XDG_CONFIG_HOME/staubli_gui/user_points.yaml  (por defecto
    ~/.config/staubli_gui/user_points.yaml).

La variable de entorno STAUBLI_GUI_CONFIG_DIR permite redirigir ese
directorio (p. ej. en pruebas automatizadas sin tocar el home real).
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import PyKDL as kdl
import yaml

# ─── Localización del fichero de puntos ────────────────────────────────


def default_config_dir() -> Path:
    """Directorio configuración del usuario (override con env para pruebas)."""
    override = os.environ.get("STAUBLI_GUI_CONFIG_DIR", "").strip()
    if override:
        return Path(override)
    xdg = os.environ.get("XDG_CONFIG_HOME", "").strip()
    base = Path(xdg) if xdg else Path.home() / ".config"
    return base / "staubli_gui"


def default_config_path() -> Path:
    """Ruta del YAML de puntos del usuario."""
    return default_config_dir() / "user_points.yaml"


# ─── Conversión de orientación ─────────────────────────────────────────


def rpy_degrees_to_quaternion_xyzw(
    rx_deg: float, ry_deg: float, rz_deg: float
) -> List[float]:
    """Convierte RPY en grados a cuaternión (x, y, z, w) con la convención KDL.

    Usa PyKDL (misma librería que la IK del proyecto) para no introducir una
    convención propia: kdl.Rotation.RPY(roll, pitch, yaw) construye
    R = Rz(yaw)·Ry(pitch)·Rx(roll) y GetQuaternion() devuelve (x, y, z, w).
    """
    rx, ry, rz = (math.radians(float(v)) for v in (rx_deg, ry_deg, rz_deg))
    return [float(v) for v in kdl.Rotation.RPY(rx, ry, rz).GetQuaternion()]


# ─── Modelo ────────────────────────────────────────────────────────────


@dataclass
class UserPoint:
    """Pose cartesiana de tool0 + configuración articular calculada."""

    name: str
    position_xyz: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpy_degrees: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    quaternion_xyzw: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])
    q: List[float] = field(default_factory=lambda: [0.0] * 6)
    solved: bool = False
    inside_limits: bool = False


def _to_float_list(value, length: int) -> Optional[List[float]]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        return None
    try:
        return [float(v) for v in value]
    except (TypeError, ValueError):
        return None


def _userpoint_from_dict(data) -> Optional[UserPoint]:
    """Construye un UserPoint a partir de un dict YAML (tolerante)."""
    if not isinstance(data, dict):
        return None
    name = data.get("name")
    pos = _to_float_list(data.get("position"), 3)
    q = _to_float_list(data.get("q"), 6)
    if not name or pos is None or q is None:
        return None
    rpy = _to_float_list(data.get("rpy_degrees"), 3)
    xyzw = _to_float_list(data.get("quaternion_xyzw"), 4)
    if rpy is None or xyzw is None:
        return None
    return UserPoint(
        name=str(name),
        position_xyz=pos,
        rpy_degrees=rpy,
        quaternion_xyzw=xyzw,
        q=q,
        solved=bool(data.get("solved", True)),
        inside_limits=bool(data.get("inside_limits", True)),
    )


def _userpoint_to_dict(point: UserPoint) -> Dict[str, object]:
    return {
        "name": point.name,
        "position": [float(v) for v in point.position_xyz],
        "rpy_degrees": [float(v) for v in point.rpy_degrees],
        "quaternion_xyzw": [float(v) for v in point.quaternion_xyzw],
        "q": [float(v) for v in point.q],
        "solved": bool(point.solved),
        "inside_limits": bool(point.inside_limits),
    }


# ─── Almacén ───────────────────────────────────────────────────────────


class UserPointStore:
    """Carga/guardado del YAML de puntos del usuario (escritura atómica)."""

    def __init__(self, path: Optional[Path] = None) -> None:
        self.path = Path(path) if path is not None else default_config_path()
        self._points: Dict[str, UserPoint] = {}
        self.errors: List[str] = []
        self.load()

    # ------------------------------------------------------------------
    def load(self) -> None:
        """Recarga desde disco; entradas inválidas se omiten y se anotan."""
        self._points = {}
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
        for i, entry in enumerate(raw.get("points", []) or []):
            point = _userpoint_from_dict(entry)
            if point is None:
                self.errors.append(f"entrada {i + 1} inválida (omitida).")
                continue
            if point.name in self._points:
                self.errors.append(
                    f"nombre duplicado '{point.name}' (se mantiene el primero)."
                )
                continue
            self._points[point.name] = point

    # ------------------------------------------------------------------
    def save(self) -> None:
        """Guarda el fichero en una escritura atómica (temp + os.replace)."""
        self.path.parent.mkdir(parents=True, exist_ok=True)
        data = {"points": [_userpoint_to_dict(p) for p in self.all()]}
        text = yaml.safe_dump(data, sort_keys=False, allow_unicode=True)
        tmp = self.path.with_name(self.path.name + ".tmp")
        tmp.write_text(text, encoding="utf-8")
        os.replace(tmp, self.path)

    # ------------------------------------------------------------------
    def names(self) -> List[str]:
        return list(self._points)

    def all(self) -> List[UserPoint]:
        return list(self._points.values())

    def get(self, name: str) -> Optional[UserPoint]:
        return self._points.get(name)

    def has(self, name: str) -> bool:
        return name in self._points

    # ------------------------------------------------------------------
    def add(self, point: UserPoint) -> None:
        if point.name in self._points:
            raise ValueError(f"ya existe un punto llamado '{point.name}'.")
        self._points[point.name] = point

    def update(self, point: UserPoint, previous_name: Optional[str] = None) -> None:
        """Sustituye un punto; soporta el renombrado si 'previous_name' difiere."""
        if previous_name and previous_name != point.name:
            if previous_name not in self._points:
                raise ValueError(f"el punto '{previous_name}' no existe.")
            self._points.pop(previous_name)
        if point.name in self._points and point.name != previous_name:
            raise ValueError(f"ya existe un punto llamado '{point.name}'.")
        self._points[point.name] = point

    def remove(self, name: str) -> None:
        if name not in self._points:
            raise KeyError(f"el punto '{name}' no existe.")
        self._points.pop(name)