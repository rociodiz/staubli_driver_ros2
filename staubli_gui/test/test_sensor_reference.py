"""Tests de la referencia verde (sensor_reference).

El módulo es geometría pura: no requiere ROS ni Gazebo. Se comprueba que
   1. la referencia es el ancho REAL del abanico FOV del STL a la distancia de
      trabajo, en su plano físico (sensor_support_link),
   2. el segmento une las dos aristas del abanico (llega a ambos extremos),
   3. camera_link queda alineada con el boresight (mismo +X que el soporte),
   4. la proyección pinhole respeta la posición real de la cámara,
   5. el defecto coincide con el de la trayectoria (scan_path) para que no
      diverjan.
"""

import math
import re
from pathlib import Path

import pytest
import PyKDL as kdl

from staubli_gui import scan_path
from staubli_gui import sensor_reference as sr
from staubli_gui.scan_path import DEFAULT_SCAN_DISTANCE

# Xacro de la cámara (fuente de la visualización 3D). No puede importar Python,
# así que sus constantes se duplican; estos tests evitan que diverjan.
_CAMERA_XACRO = (
    Path(__file__).resolve().parents[2]
    / "staubli_robot_description"
    / "urdf"
    / "include"
    / "staubli_camera.xacro"
)


def _xacro_text():
    if not _CAMERA_XACRO.is_file():
        pytest.skip(f"no se encontró {_CAMERA_XACRO}")
    return _CAMERA_XACRO.read_text(encoding="utf-8")


def _xacro_property(text, name):
    m = re.search(rf'name="{name}"\s+value="([^"]+)"', text)
    assert m is not None, f"falta la propiedad xacro '{name}'"
    return float(m.group(1))



# ── Ancho real del abanico FOV ────────────────────────────────────────────


def test_el_ancho_en_x1_coincide_con_la_arista_del_stl():
    """A x = 464.813 mm el ancho es el del borde del trapecio (852.28 mm)."""
    assert sr.fov_width_m(sr.FOV_X1_M) == pytest.approx(
        sr.FOV_Z_HI1_M - sr.FOV_Z_LO1_M, abs=1e-12
    )


def test_el_ancho_a_la_distancia_de_trabajo_es_el_del_stl():
    """A 325 mm el abanico mide ~637.7 mm (regresión medida del STL)."""
    assert sr.fov_width_m(0.325) == pytest.approx(0.6377120675, abs=1e-9)


def test_el_ancho_crece_con_la_distancia():
    anchos = [sr.fov_width_m(d) for d in (0.10, 0.20, 0.325, 0.46481345)]
    assert anchos == sorted(anchos)
    assert anchos[-1] > anchos[0]


def test_una_distancia_no_positiva_se_rechaza():
    with pytest.raises(ValueError):
        sr.fov_width_m(0.0)
    with pytest.raises(ValueError):
        sr.fov_width_m(-0.1)


# ── El segmento une las dos aristas, en el plano del FOV ──────────────────


def test_el_segmento_une_las_dos_aristas_del_abanico():
    p_lo, p_hi = sr.working_reference_segment_sensor(0.325)
    # Ambos en la distancia pedida y en el plano físico del abanico.
    assert p_lo[0] == pytest.approx(0.325)
    assert p_hi[0] == pytest.approx(0.325)
    assert p_lo[1] == pytest.approx(sr.FOV_PLANE_Y_M)
    assert p_hi[1] == pytest.approx(sr.FOV_PLANE_Y_M)
    # Separados exactamente por el ancho real del FOV.
    assert (p_hi[2] - p_lo[2]) == pytest.approx(sr.fov_width_m(0.325))
    # Centrados en el eje del abanico (z ≈ +14.688 mm), no en z = 0.
    centro = sr.working_reference_center_sensor(0.325)
    assert centro[2] == pytest.approx(0.01468785, abs=1e-6)


def test_el_segmento_es_configurable():
    p_lo, p_hi = sr.working_reference_segment_sensor(0.25)
    assert p_lo[0] == pytest.approx(0.25)
    assert (p_hi[2] - p_lo[2]) == pytest.approx(sr.fov_width_m(0.25))


# ── camera_link alineada con el boresight ─────────────────────────────────


def test_camera_link_apunta_a_lo_largo_del_boresight():
    """El +X de camera_link coincide con el +X del soporte (sin yaw de 19°)."""
    mc = sr.camera_mount_frame()
    boresight = mc.M * kdl.Vector(1.0, 0.0, 0.0)
    assert [boresight[i] for i in range(3)] == pytest.approx([1.0, 0.0, 0.0], abs=1e-9)


def test_el_origen_de_camera_link_no_se_mueve():
    """El origen físico del sensor se mantiene (solo cambia la orientación)."""
    assert sr.CAMERA_MOUNT_XYZ == (-0.064522, 0.088917, 0.014688)


def test_el_montaje_del_sensor_es_el_de_scan_path():
    """No se duplica el montaje: se usa el mismo que scan_path."""
    assert sr.SENSOR_MOUNT_XYZ == scan_path.SENSOR_MOUNT_XYZ
    assert sr.SENSOR_MOUNT_RPY == scan_path.SENSOR_MOUNT_RPY
    assert sr.FOV_FRAME == scan_path.SENSOR_FRAME


# ── Proyección pinhole ────────────────────────────────────────────────────


def test_la_focal_deriva_del_hfov():
    f = sr.focal_pixels(640, 1.0472)
    assert f == pytest.approx((640 / 2) / math.tan(1.0472 / 2))


def test_el_punto_del_eje_optico_cae_en_el_centro_de_la_imagen():
    u, v = sr.project_point([0.3, 0.0, 0.0])
    assert u == pytest.approx(sr.CAMERA_WIDTH / 2)
    assert v == pytest.approx(sr.CAMERA_HEIGHT / 2)


def test_un_punto_detras_del_sensor_se_rechaza():
    with pytest.raises(ValueError):
        sr.project_point([-0.1, 0.0, 0.0])


def test_el_segmento_proyectado_es_horizontal():
    """El plano del FOV es paralelo al eje de la cámara: misma fila v.

    La fila coincide salvo el redondeo del rpy del montaje en el xacro
    (-1.570796 en vez de -π/2), sub-píxel.
    """
    (u0, v0), (u1, v1) = sr.working_reference_pixels(0.325)
    assert v0 == pytest.approx(v1, abs=1e-3)
    assert u0 < sr.CAMERA_WIDTH / 2 < u1


def test_la_longitud_proyectada_crece_al_acercarse():
    cerca = sr.working_reference_pixels(0.20)
    lejos = sr.working_reference_pixels(0.50)
    ancho_cerca = abs(cerca[1][0] - cerca[0][0])
    ancho_lejos = abs(lejos[1][0] - lejos[0][0])
    assert ancho_cerca > ancho_lejos > 0


# ── Coherencia con scan_path ──────────────────────────────────────────────


def test_el_defecto_coincide_con_el_de_la_trayectoria():
    """La referencia verde y la trayectoria no deben divergir en su defecto."""
    assert sr.DEFAULT_WORKING_DISTANCE_M == DEFAULT_SCAN_DISTANCE


def test_el_frame_de_referencia_es_camera_link():
    assert sr.SENSOR_REFERENCE_FRAME == "camera_link"


# ── Coherencia con el xacro (misma geometría, sin divergir) ───────────────


def test_las_constantes_del_fov_coinciden_con_el_xacro():
    text = _xacro_text()
    for nombre, valor in {
        "fov_x0": sr.FOV_X0_M,
        "fov_x1": sr.FOV_X1_M,
        "fov_zlo0": sr.FOV_Z_LO0_M,
        "fov_zhi0": sr.FOV_Z_HI0_M,
        "fov_zlo1": sr.FOV_Z_LO1_M,
        "fov_zhi1": sr.FOV_Z_HI1_M,
        "fov_plane_y": sr.FOV_PLANE_Y_M,
    }.items():
        assert _xacro_property(text, nombre) == pytest.approx(valor), nombre


def test_el_montaje_de_camera_link_coincide_con_el_xacro():
    text = _xacro_text()
    m = re.search(
        r'<joint name="\$\{robot_prefix\}camera_fixed_joint".*?'
        r'<origin xyz="([^"]+)" rpy="([^"]+)"',
        text,
        re.DOTALL,
    )
    assert m is not None, "no se encontró camera_fixed_joint en el xacro"
    xyz = [float(v) for v in m.group(1).split()]
    rpy = [float(v) for v in m.group(2).split()]
    assert xyz == pytest.approx(list(sr.CAMERA_MOUNT_XYZ))
    assert rpy == pytest.approx(list(sr.CAMERA_MOUNT_RPY))


def test_el_montaje_del_sensor_coincide_con_el_xacro():
    text = _xacro_text()
    m = re.search(
        r'<joint name="\$\{robot_prefix\}sensor_support_joint".*?'
        r'<origin xyz="([^"]+)" rpy="([^"]+)"',
        text,
        re.DOTALL,
    )
    assert m is not None, "no se encontró sensor_support_joint en el xacro"
    xyz = [float(v) for v in m.group(1).split()]
    rpy = [float(v) for v in m.group(2).split()]
    assert xyz == pytest.approx(list(scan_path.SENSOR_MOUNT_XYZ))
    assert rpy == pytest.approx(list(scan_path.SENSOR_MOUNT_RPY))

