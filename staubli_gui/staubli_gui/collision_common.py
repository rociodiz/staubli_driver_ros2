"""Validación geométrica aproximada centralizada para sensor vs mesa/pieza.

Este módulo SOLO centraliza la comprobación aproximada del sensor frente a
la mesa y la pieza (tal como se hacía en scan_path). No es un sistema general
de prevención de colisiones del robot completo.
"""

from __future__ import annotations

from typing import Tuple

import PyKDL as kdl

# Geometría tomada de inspection_cell.sdf / PartBox / montaje sensor
TABLE_TOP_Z_BASELINK: float = 0.20
TABLE_HALF_X: float = 0.5
TABLE_HALF_Y: float = 0.3
TABLE_CENTER_Y: float = 0.55
TABLE_CLEARANCE_M: float = 0.005

# AABB local del sensor (SENSOR_BASE_collision.stl) en metros
SENSOR_COLLISION_LOCAL_MIN = (-0.17668630981445313, -0.03810824203491211, -0.014362157821655274)
SENSOR_COLLISION_LOCAL_MAX = (-0.05268631362915039, 0.16739175415039062, 0.046637840270996093)


class PartBox:
    def __init__(self, center=None, size=None):
        from . import scan_path

        if center is None:
            center = list(scan_path.DEFAULT_PART_CENTER)
        if size is None:
            size = list(scan_path.DEFAULT_PART_SIZE)
        self.center = [float(v) for v in center]
        self.size = [float(v) for v in size]


def _aabb_intersect(a_min, a_max, b_min, b_max, margin: float = 0.0) -> bool:
    for i in range(3):
        if a_max[i] + margin < b_min[i] - margin or b_max[i] + margin < a_min[i] - margin:
            return False
    return True


def sensor_oriented_aabb(sensor_frame_world: kdl.Frame):
    mn = SENSOR_COLLISION_LOCAL_MIN
    mx = SENSOR_COLLISION_LOCAL_MAX
    xs, ys, zs = [], [], []
    for dx in (mn[0], mx[0]):
        for dy in (mn[1], mx[1]):
            for dz in (mn[2], mx[2]):
                p = sensor_frame_world * kdl.Vector(dx, dy, dz)
                xs.append(p[0])
                ys.append(p[1])
                zs.append(p[2])
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def table_aabb():
    return (-TABLE_HALF_X, TABLE_CENTER_Y - TABLE_HALF_Y, 0.0), (TABLE_HALF_X, TABLE_CENTER_Y + TABLE_HALF_Y, TABLE_TOP_Z_BASELINK)


def part_aabb(part):
    # Acepta PartBox de scan_path o nuestro wrapper
    if hasattr(part, "center") and hasattr(part, "size"):
        c = part.center
        s = part.size
    else:
        c = getattr(part, "center", [0.0, 0.46, 0.24])
        s = getattr(part, "size", [0.08, 0.08, 0.08])
    return (c[0] - s[0] / 2.0, c[1] - s[1] / 2.0, c[2] - s[2] / 2.0), (c[0] + s[0] / 2.0, c[1] + s[1] / 2.0, c[2] + s[2] / 2.0)


def check_sensor_environment_collision(sensor_frame_world: kdl.Frame, part, clearance: float = TABLE_CLEARANCE_M) -> Tuple[bool, str]:
    """Comprueba colisión aproximada sensor vs mesa/pieza."""
    smin, smax = sensor_oriented_aabb(sensor_frame_world)
    tmin, tmax = table_aabb()
    in_table_xy = not (smax[0] < tmin[0] or smin[0] > tmax[0] or smax[1] < tmin[1] or smin[1] > tmax[1])
    in_table_xy = in_table_xy or (smin[0] > -0.6 and smax[0] < 0.6 and smin[1] > -0.1 and smax[1] < 0.95)
    s_origin = (sensor_frame_world.p[0], sensor_frame_world.p[1], sensor_frame_world.p[2])
    if smin[2] < TABLE_TOP_Z_BASELINK + clearance:
        return True, f"sensor_intersects_table (z_min={smin[2]:.4f} < {TABLE_TOP_Z_BASELINK + clearance:.4f})"
    if s_origin[2] < TABLE_TOP_Z_BASELINK + clearance:
        if abs(s_origin[0]) < 0.8 or s_origin[1] < 0.9 or in_table_xy:
            return True, f"sensor_origin_below_table ({s_origin[2]:.4f})"
    if _aabb_intersect(smin, smax, tmin, tmax, margin=0.0):
        return True, "sensor_intersects_table_volume"
    pmin, pmax = part_aabb(part)
    if _aabb_intersect(smin, smax, pmin, pmax, margin=0.0):
        return True, "sensor_intersects_part"
    return False, ""
