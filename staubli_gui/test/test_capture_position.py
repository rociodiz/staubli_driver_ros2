"""Tests headless de la captura de la posición actual (GUARDAR POSICIÓN ACTUAL).

No se mueve ningún robot ni se escribe en el home real: conftest.py aísla
STAUBLI_GUI_CONFIG_DIR en un directorio temporal y las q usadas son valores
reales leídos de /joint_states (sin ROS).

Cubre el helper puro user_point_from_joint_position, la persistencia en el
mismo user_points.yaml de Puntos del usuario y el flujo completo del botón:
capturar -> guardar -> aparecer en la lista -> usar "MOVER A PUNTO" (goal) y
"AÑADIR A TRAYECTORIA".
"""

import pytest

import staubli_gui.user_points as up

# J1..J6 reales (lectura de /joint_states) usados como 'posición actual'.
_Q_REAL = [
    0.980522599, 0.654816443, 1.840180676,
    1.417612398, 1.188560861, -2.477094896,
]
# Pose FK esperada de tool0 para _Q_REAL (base_link, ver tx2_60l_ik).
_Q_REAL_POS = [0.220897, 0.480977, 0.306291]
_Q_REAL_RPY = [-175.0444, 67.3914, -35.401]
_Q_HOME = [0.0] * 6
_Q_FUERA = [0.0, 2.35, 0.0, 0.0, 0.0, 0.0]  # J2 > 127° -> fuera de límites


def _kin():
    from staubli_trajectory.tx2_60l_ik import Tx260lKinematics

    return Tx260lKinematics()


def _set_joints_now(win, q):
    win._joints_now = {f"joint_{i}": v for i, v in enumerate(q, start=1)}


@pytest.fixture
def win(config_dir, qapp):
    """Ventana de la GUI sin ROS (layout/widgets reales, sin puntos previos)."""
    from staubli_gui.gui import MainWindow

    window = MainWindow(ros_node=None)
    yield window
    window.close()


# ── Helper user_point_from_joint_position (FK sin IK inversa) ───────────


def test_helper_home_sin_movimiento():
    p = up.user_point_from_joint_position("Punto 1", _Q_HOME, _kin())
    assert p.position_xyz == pytest.approx([0.0, 0.02, 1.295], abs=1e-6)
    assert p.rpy_degrees == pytest.approx([0.0, -0.0, 0.0], abs=1e-3)
    assert p.quaternion_xyzw == pytest.approx([0.0, 0.0, 0.0, 1.0], abs=1e-6)
    assert list(p.q) == pytest.approx(_Q_HOME)
    assert p.solved is True
    assert p.inside_limits is True


def test_helper_pose_real_coincide_con_fk():
    p = up.user_point_from_joint_position("Punto 1", _Q_REAL, _kin())
    assert p.position_xyz == pytest.approx(_Q_REAL_POS, abs=1e-6)
    assert p.rpy_degrees == pytest.approx(_Q_REAL_RPY, abs=1e-3)
    assert list(p.q) == pytest.approx(_Q_REAL)  # la q real se guarda tal cual
    assert p.solved is True
    assert p.inside_limits is True


def test_helper_quaternion_consistente_con_rpy():
    p = up.user_point_from_joint_position("Punto 1", _Q_REAL, _kin())
    rquat = up.rpy_degrees_to_quaternion_xyzw(*p.rpy_degrees)
    assert p.quaternion_xyzw == pytest.approx(rquat, abs=1e-6)


def test_helper_marca_fuera_de_limites():
    p = up.user_point_from_joint_position("Punto 1", _Q_FUERA, _kin())
    assert p.inside_limits is False
    assert len(p.q) == 6


def test_helper_rechaza_q_incompleta():
    with pytest.raises(ValueError):
        up.user_point_from_joint_position("Punto 1", [0.0, 0.0], _kin())


# ── Flujo de almacenamiento (mismo user_points.yaml de Puntos del usuario) ──


def test_almacenamiento_roundtrip(tmp_path):
    path = tmp_path / "user_points.yaml"
    store = up.UserPointStore(path)
    store.add(up.user_point_from_joint_position("Punto 1", _Q_REAL, _kin()))
    store.save()
    assert path.is_file()

    reloaded = up.UserPointStore(path)
    assert reloaded.names() == ["Punto 1"]
    p = reloaded.get("Punto 1")
    assert list(p.q) == pytest.approx(_Q_REAL)
    assert p.position_xyz == pytest.approx(_Q_REAL_POS, abs=1e-6)
    assert p.rpy_degrees == pytest.approx(_Q_REAL_RPY, abs=1e-3)
    assert p.solved is True and p.inside_limits is True


def test_almacenamiento_añadido_a_puntos_existentes(tmp_path):
    path = tmp_path / "user_points.yaml"
    store = up.UserPointStore(path)
    existente = up.UserPoint(
        name="Punto 1", q=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6], solved=True, inside_limits=True
    )
    store.add(existente)
    store.save()
    store.add(up.user_point_from_joint_position("Punto 2", _Q_REAL, _kin()))
    store.save()
    reloaded = up.UserPointStore(path)
    assert reloaded.names() == ["Punto 1", "Punto 2"]  # no borra nada previo


# ── Flujo en la GUI: botón GUARDAR POSICIÓN ACTUAL ──────────────────────


def test_gui_captura_crea_punto_en_lista_y_seleccionado(win):
    _set_joints_now(win, _Q_REAL)
    win._on_capture_position()
    assert win._store.has("Punto 1")
    assert win._store.names() == ["Punto 1"]
    textos = [win._points_list.item(i).text() for i in range(win._points_list.count())]
    assert "Punto 1" in textos
    assert win._points_list.currentRow() == len(textos) - 1  # queda seleccionado


def test_gui_captura_se_puede_mover_a_punto(win):
    _set_joints_now(win, _Q_REAL)
    win._on_capture_position()
    point = win._store.get("Punto 1")
    goal = win._build_single_goal(point.q, 10.0)
    assert list(goal.trajectory.points[0].positions) == pytest.approx(_Q_REAL)


def test_gui_captura_no_añade_a_trayectoria_automaticamente(win):
    _set_joints_now(win, _Q_REAL)
    win._on_capture_position()
    assert win._user_trajectory == []
    assert win._trajectory_list.count() == 0


def test_gui_capturado_se_puede_añadir_a_trayectoria(win):
    _set_joints_now(win, _Q_REAL)
    win._on_capture_position()
    win._on_add_to_trajectory()  # funcionalidad existente
    assert win._user_trajectory == ["Punto 1"]


def test_gui_nombres_secuenciales(win):
    _set_joints_now(win, _Q_REAL)
    win._on_capture_position()
    _set_joints_now(win, _Q_HOME)
    win._on_capture_position()
    assert win._store.names() == ["Punto 1", "Punto 2"]


def test_gui_sin_joint_states_no_guarda(win):
    win._joints_now = None
    win._on_capture_position()
    assert not win._store.has("Punto 1")