"""Referencia visual de la distancia de trabajo del sensor eye-in-hand.

Este módulo es PURO (sin ROS ni Qt): describe el ancho REAL del abanico FOV del
sensor a la distancia de trabajo, en su plano físico, y cómo se proyecta en la
imagen de ``/camera/image_raw``. Lo usan la GUI (para superponer la línea verde)
y los tests, de forma que la geometría es verificable sin lanzar Gazebo.

Punto de referencia físico
--------------------------
El boresight del sensor es el ``+X`` de ``sensor_support_link`` (el mismo eje
que usa ``scan_path`` para situar el sensor y medir ``scan_distance``). El FOV
real del sensor es el abanico translúcido de ``SENSOR_BASE_plane.stl``, colgado
de ese link: un trapecio contenido en el plano ``y = FOV_PLANE_Y_M`` que se abre
a lo largo de ``+X`` y cuyo ancho se mide sobre ``Z``.

La referencia verde es el segmento que corta ese trapecio a ``x = distance``:

    P_lo = (distance, FOV_PLANE_Y_M, z_lo(distance))
    P_hi = (distance, FOV_PLANE_Y_M, z_hi(distance))

y "llega a ambos extremos del abanico" por construcción: sus extremos son las
dos aristas del FOV a esa distancia.

Convención geométrica compartida con ``scan_path``
--------------------------------------------------
El montaje ``tool0 -> sensor_support_link`` (``SENSOR_MOUNT_*``) y la distancia
de trabajo por defecto (``DEFAULT_SCAN_DISTANCE``) se importan de ``scan_path``
para que no divergan. El montaje ``sensor_support_link -> camera_link`` es el
del ``camera_fixed_joint`` del xacro, con la orientación ALINEADA al boresight:
su ``+X`` coincide con el ``+X`` del soporte (se quitó el desvío en yaw de
~19°, ``-0.331612 rad``). El origen NO se toca.

Proyección pinhole
------------------
El sensor RGB de Gazebo mira a lo largo del ``+X`` de ``camera_link``. Con
píxeles cuadrados (``f = f_x = f_y``) y origen de imagen en la esquina superior
izquierda:

    f  = (width / 2) / tan(hfov / 2)
    u  = width / 2  - f · y / x
    v  = height / 2 - f · z / x

Los puntos se transforman primero de ``sensor_support_link`` a ``camera_link``
con el montaje físico (su inverso), de modo que la proyección respeta la
posición real de la cámara.
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import PyKDL as kdl

from .scan_path import (
    DEFAULT_SCAN_DISTANCE,
    SENSOR_FRAME,
    SENSOR_MOUNT_RPY,
    SENSOR_MOUNT_XYZ,
)

# ── Modelo de la cámara RGB (espejo de staubli_camera.xacro) ──────────────
# Sensor gz: 640×480, horizontal_fov = 1.0472 rad (≈60°), píxeles cuadrados.
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_HFOV_RAD = 1.0472

# Frame físico del que se mide la distancia de trabajo (eje óptico = +X).
SENSOR_REFERENCE_FRAME = "camera_link"

# Frame del sensor (y del abanico FOV): el mismo que usa scan_path.
FOV_FRAME = SENSOR_FRAME

# ── Geometría del abanico FOV (SENSOR_BASE_plane.stl, en sensor_support_link) ─
# Trapecio re-centrado en el plano y = FOV_PLANE_Y_M = 0 (boresight): el STL se
# exportó a y ≈ -19.82 mm, pero el montaje lo re-centra para que el abanico (y
# la referencia verde) caiga sobre el eje del sensor. Aristas (x, z), en metros,
# extraídas de la malla (1108 triángulos). El vértice virtual cae en x ≈ -0.0905 m.
FOV_PLANE_Y_M = 0.0
FOV_X0_M = -0.07018654
FOV_X1_M = 0.46481345
FOV_Z_LO0_M = -0.00093088686  # arista inferior en x0
FOV_Z_HI0_M = 0.03030657      # arista superior en x0
FOV_Z_LO1_M = -0.41145084     # arista inferior en x1
FOV_Z_HI1_M = 0.4408265       # arista superior en x1

# ── Montaje sensor_support_link -> camera_link (camera_fixed_joint) ───────
# Orientación alineada con el boresight: rpy = (-π/2, 0, 0) (sin el yaw
# de -0.331612 rad). Se mantiene intacto el origen físico del sensor.
CAMERA_MOUNT_XYZ: Tuple[float, float, float] = (-0.064522, 0.088917, 0.014688)
CAMERA_MOUNT_RPY: Tuple[float, float, float] = (-1.570796, 0.0, 0.0)

# ── Distancia de trabajo por defecto ──────────────────────────────────────
# Es el MISMO parámetro "distancia óptima de trabajo" que scan_path: la GUI lo
# mantiene configurable y la referencia verde lo sigue.
DEFAULT_WORKING_DISTANCE_M = DEFAULT_SCAN_DISTANCE


def focal_pixels(
    width: int = CAMERA_WIDTH,
    hfov_rad: float = CAMERA_HFOV_RAD,
) -> float:
    """Distancia focal en píxeles (píxeles cuadrados) a partir del hfov."""
    if width <= 0:
        raise ValueError("width debe ser positivo.")
    if not 0.0 < hfov_rad < math.pi:
        raise ValueError("hfov_rad debe estar en (0, pi).")
    return (width / 2.0) / math.tan(hfov_rad / 2.0)


def sensor_mount_frame() -> kdl.Frame:
    """Pose de ``sensor_support_link`` respecto a ``tool0`` (para completitud)."""
    return kdl.Frame(
        kdl.Rotation.RPY(*SENSOR_MOUNT_RPY),
        kdl.Vector(*SENSOR_MOUNT_XYZ),
    )


def camera_mount_frame() -> kdl.Frame:
    """Pose de ``camera_link`` respecto a ``sensor_support_link`` (alineada)."""
    return kdl.Frame(
        kdl.Rotation.RPY(*CAMERA_MOUNT_RPY),
        kdl.Vector(*CAMERA_MOUNT_XYZ),
    )


def _edge_z(z0: float, z1: float, x: float) -> float:
    """Interpola el borde del abanico (recta de (x0,z0)-(x1,z1)) en ``x``."""
    if x <= 0.0:
        raise ValueError("distance_m debe ser positiva.")
    t = (x - FOV_X0_M) / (FOV_X1_M - FOV_X0_M)
    return z0 + t * (z1 - z0)


def fov_width_m(distance_m: float) -> float:
    """Ancho REAL del FOV (m) a ``distance_m`` del sensor, en el plano del abanico."""
    d = float(distance_m)
    if d <= 0.0:
        raise ValueError("distance_m debe ser positiva.")
    z_lo = _edge_z(FOV_Z_LO0_M, FOV_Z_LO1_M, d)
    z_hi = _edge_z(FOV_Z_HI0_M, FOV_Z_HI1_M, d)
    return z_hi - z_lo


def working_reference_segment_sensor(
    distance_m: float,
) -> Tuple[List[float], List[float]]:
    """Extremos 3D de la línea verde, en el frame del SENSOR (plano del FOV).

    El segmento corta el abanico a ``x = distance_m`` y une sus dos aristas, así
    que su longitud es exactamente ``fov_width_m(distance_m)``.
    """
    d = float(distance_m)
    if d <= 0.0:
        raise ValueError("distance_m debe ser positiva.")
    z_lo = _edge_z(FOV_Z_LO0_M, FOV_Z_LO1_M, d)
    z_hi = _edge_z(FOV_Z_HI0_M, FOV_Z_HI1_M, d)
    return ([d, FOV_PLANE_Y_M, z_lo], [d, FOV_PLANE_Y_M, z_hi])


def working_reference_center_sensor(distance_m: float) -> List[float]:
    """Centro del abanico a ``distance_m``, en el frame del SENSOR."""
    p_lo, p_hi = working_reference_segment_sensor(distance_m)
    return [(p_lo[i] + p_hi[i]) / 2.0 for i in range(3)]


def _to_camera(point_sensor: Sequence[float]) -> List[float]:
    """Transforma un punto del frame del SENSOR al frame de ``camera_link``."""
    p = camera_mount_frame().Inverse() * kdl.Vector(
        float(point_sensor[0]), float(point_sensor[1]), float(point_sensor[2])
    )
    return [float(p[0]), float(p[1]), float(p[2])]


def working_reference_segment_camera(
    distance_m: float,
) -> Tuple[List[float], List[float]]:
    """Extremos de la línea verde en el frame de ``camera_link``."""
    p_lo, p_hi = working_reference_segment_sensor(distance_m)
    return (_to_camera(p_lo), _to_camera(p_hi))


def working_reference_center_camera(distance_m: float) -> List[float]:
    """Centro del abanico en el frame de ``camera_link``."""
    return _to_camera(working_reference_center_sensor(distance_m))


def project_point(
    point_xyz: Sequence[float],
    width: int = CAMERA_WIDTH,
    height: int = CAMERA_HEIGHT,
    hfov_rad: float = CAMERA_HFOV_RAD,
) -> Tuple[float, float]:
    """Proyecta un punto en el frame de ``camera_link`` a píxeles ``(u, v)``.

    Usa el modelo pinhole con píxeles cuadrados descrito en el docstring del
    módulo. Un punto sobre el eje óptico cae en el centro de la imagen.
    """
    x, y, z = (float(c) for c in point_xyz)
    if x <= 0.0:
        raise ValueError("el punto debe estar delante del sensor (x > 0).")
    if height <= 0:
        raise ValueError("height debe ser positivo.")
    focal = focal_pixels(width, hfov_rad)
    u = width / 2.0 - focal * (y / x)
    v = height / 2.0 - focal * (z / x)
    return (u, v)


def working_reference_pixels(
    distance_m: float,
    width: int = CAMERA_WIDTH,
    height: int = CAMERA_HEIGHT,
    hfov_rad: float = CAMERA_HFOV_RAD,
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """Extremos en píxeles de la línea verde para ``/camera/image_raw``.

    Proyecta el segmento del abanico (``working_reference_segment_camera``) con
    los intrínsecos de la cámara. Los extremos pueden caer fuera del marco si el
    abanico es más ancho que el FOV de la cámara; la GUI recorta al pintar.
    """
    p0, p1 = working_reference_segment_camera(distance_m)
    return (
        project_point(p0, width, height, hfov_rad),
        project_point(p1, width, height, hfov_rad),
    )
