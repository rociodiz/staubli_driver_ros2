"""Tests headless del mando virtual (joystick) de staubli_gui.

No hay ROS ni robot: se contrastan la matemática del stick (deadzone,
redondeo al círculo, convención de ejes), el mapeo a componentes de un
Twist (map_twist_components) y el comportamiento del JoystickPanel
(readout, reset, ganancia). Se usa QT_QPA_PLATFORM=offscreen (conftest).
"""

import pytest

from staubli_gui.joystick_widgets import (
    JoystickPanel,
    VirtualJoystick,
    _aplicar_deadzone,
    _limitar_a_circulo,
    map_twist_components,
)


# ── Geometría pura ──────────────────────────────────────────────────────

def test_deadzone_borra_comandos_pequenos():
    x, y = _aplicar_deadzone(0.05, 0.05, 0.15)
    assert x == 0.0 and y == 0.0


def test_deadzone_deja_escapar_todo_el_recorrido():
    x, y = _aplicar_deadzone(1.0, 0.0, 0.15)
    assert x == pytest.approx(1.0)
    assert y == pytest.approx(0.0)


def test_redondeo_al_circulo_unitario():
    x, y = _limitar_a_circulo(3.0, 4.0)  # magnitud 5 -> (0.6, 0.8)
    assert x == pytest.approx(0.6)
    assert y == pytest.approx(0.8)


# ── Mapeo de ejes (función pura) ───────────────────────────────────────

def test_mapeo_inicial_izq_lineal():
    c = map_twist_components(izq_x=1.0, izq_y=0.5, der_x=0.0, der_y=0.0, z_val=0.0)
    assert c["linear"]["x"] == pytest.approx(1.0)
    assert c["linear"]["y"] == pytest.approx(0.5)
    assert c["linear"]["z"] == pytest.approx(0.0)


def test_mapeo_der_angular_y_angular_x_reservado():
    c = map_twist_components(izq_x=0.0, izq_y=0.0, der_x=-0.8, der_y=0.3, z_val=0.0)
    assert c["angular"]["x"] == pytest.approx(0.0)  # reservado, sin asignar
    assert c["angular"]["y"] == pytest.approx(0.3)  # der.y -> angular.y
    assert c["angular"]["z"] == pytest.approx(-0.8)  # der.x -> angular.z


def test_botones_z_via_z_val():
    c = map_twist_components(0.0, 0.0, 0.0, 0.0, z_val=1.0)
    assert c["linear"]["z"] == pytest.approx(1.0)
    c = map_twist_components(0.0, 0.0, 0.0, 0.0, z_val=-1.0)
    assert c["linear"]["z"] == pytest.approx(-1.0)


def test_ganancia_multiplica_todos_los_componentes():
    c = map_twist_components(1.0, 0.5, -0.25, 0.75, z_val=1.0, gain=2.0)
    assert c["linear"]["x"] == pytest.approx(2.0)
    assert c["linear"]["y"] == pytest.approx(1.0)
    assert c["linear"]["z"] == pytest.approx(2.0)
    assert c["angular"]["x"] == pytest.approx(0.0)
    assert c["angular"]["y"] == pytest.approx(1.5)
    assert c["angular"]["z"] == pytest.approx(-0.5)


# ── Widget VirtualJoystick ──────────────────────────────────────────────

def test_stick_centro_devuelve_cero(qapp):
    stick = VirtualJoystick()
    stick.set_position(0.0, 0.0)
    assert stick.value == (0.0, 0.0)


def test_stick_dentro_de_deadzone_devuelve_cero(qapp):
    stick = VirtualJoystick()
    stick.set_position(0.10, 0.0)  # deadzone 0.15
    assert stick.value == (0.0, 0.0)


def test_stick_deflexion_maxima_derecha(qapp):
    stick = VirtualJoystick()
    stick.set_position(2.0, 0.0)  # más allá del radio -> se clampa a 1
    assert stick.value[0] == pytest.approx(1.0)
    assert stick.value[1] == pytest.approx(0.0)


def test_stick_arriba_es_y_positiva(qapp):
    # set_position usa y de pantalla (hacia abajo); arriba => -1.
    stick = VirtualJoystick()
    stick.set_position(0.0, -1.0)
    assert stick.value[0] == pytest.approx(0.0)
    assert stick.value[1] == pytest.approx(1.0)


def test_stick_reset_centra(qapp):
    stick = VirtualJoystick()
    stick.set_position(0.8, -0.6)
    assert stick.value != (0.0, 0.0)
    stick.reset()
    assert stick.value == (0.0, 0.0)


# ── Panel ───────────────────────────────────────────────────────────────

@pytest.fixture
def panel(qapp):
    return JoystickPanel()


def test_panel_estado_inicial_cero(panel):
    comp = panel.twist_components()
    assert comp["linear"] == {"x": 0.0, "y": 0.0, "z": 0.0}
    assert comp["angular"] == {"x": 0.0, "y": 0.0, "z": 0.0}


def test_panel_stick_izq_mueve_linear(panel):
    panel.stick_izq.set_position(1.0, 0.0)
    panel.stick_der.set_position(0.0, 0.0)
    comp = panel.twist_components()
    assert comp["linear"]["x"] == pytest.approx(1.0)
    assert comp["linear"]["z"] == pytest.approx(0.0)


def test_panel_stick_der_mueve_angular(panel):
    panel.stick_izq.set_position(0.0, 0.0)
    panel.stick_der.set_position(0.0, -1.0)  # arriba -> +angular.y
    comp = panel.twist_components()
    assert comp["angular"]["y"] == pytest.approx(1.0)
    assert comp["angular"]["x"] == pytest.approx(0.0)


def test_panel_botones_z(panel):
    panel._zp = True
    comp = panel.twist_components()
    assert comp["linear"]["z"] == pytest.approx(1.0)
    panel._zp = False
    panel._zm = True
    comp = panel.twist_components()
    assert comp["linear"]["z"] == pytest.approx(-1.0)


def test_panel_ganancia_aplica(panel):
    panel.stick_izq.set_position(0.5, 0.0)
    panel.set_gain(2.00)
    comp = panel.twist_components()
    assert comp["linear"]["x"] == pytest.approx(1.0)  # 0.5 * 2.0


def test_panel_reset_libera_todo(panel):
    panel.stick_izq.set_position(1.0, 0.0)
    panel.stick_der.set_position(-1.0, 0.0)
    panel._zp = True
    comp = panel.twist_components()
    assert comp["linear"]["x"] != 0.0
    panel.reset()
    comp = panel.twist_components()
    assert comp["linear"] == {"x": 0.0, "y": 0.0, "z": 0.0}
    assert comp["angular"] == {"x": 0.0, "y": 0.0, "z": 0.0}


def test_panel_readout_se_actualiza(panel):
    panel.stick_izq.set_position(0.4, 0.0)
    assert panel._readout["lin_x"].text() == "0.40"


# ── Integración con MainWindow (sin ROS) ────────────────────────────────

@pytest.fixture
def win(config_dir, qapp):
    from staubli_gui.gui import MainWindow

    window = MainWindow(ros_node=None)
    yield window
    window.close()


def test_ventana_crea_panel_y_no_publica_sin_ros(win):
    assert win._joystick_panel is not None
    assert win._cmd_vel_pub is None
    assert not win._joystick_active


def test_toggle_sin_ros_es_inocuo_y_registra_log(win):
    win._joystick_panel.btn_toggle.setChecked(True)
    assert not win._joystick_active  # sin ROS: no se publica nada
    assert win._joystick_panel.btn_toggle.text() == "JOYSTICK ACTIVADO"
    assert "JOYSTICK ACTIVADO" in win._log_edit.toPlainText()