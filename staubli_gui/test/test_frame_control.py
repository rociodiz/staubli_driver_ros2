"""Tests headless del control manual FRAME de staubli_gui.

No hay ROS ni robot: se contrastan el mapeo a componentes de un Twist
(map_twist_components: X/Y/Z -> linear, RX/RY/RZ -> angular, +/-, ganancia
y velocidad) y el comportamiento del FrameControlPanel (readout, reset,
ganancia, velocidad). Se usa QT_QPA_PLATFORM=offscreen (conftest).
"""

import pytest

from staubli_gui.frame_control import FrameControlPanel, map_twist_components


# ── Mapeo de ejes (función pura) ───────────────────────────────────────

def test_mapeo_sin_botones_es_cero():
    c = map_twist_components()
    assert c == {
        "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
    }


@pytest.mark.parametrize(
    "dof,kwargs,esperado",
    [
        ("X", {"x_plus": True}, {"linear": {"x": 1.0, "y": 0.0, "z": 0.0},
                                 "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}),
        ("X-", {"x_minus": True}, {"linear": {"x": -1.0, "y": 0.0, "z": 0.0},
                                   "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}),
        ("Y", {"y_plus": True}, {"linear": {"x": 0.0, "y": 1.0, "z": 0.0},
                                 "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}),
        ("Z", {"z_plus": True}, {"linear": {"x": 0.0, "y": 0.0, "z": 1.0},
                                 "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}),
        ("RX", {"rx_plus": True}, {"linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                                   "angular": {"x": 1.0, "y": 0.0, "z": 0.0}}),
        ("RY-", {"ry_minus": True}, {"linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                                     "angular": {"x": 0.0, "y": -1.0, "z": 0.0}}),
        ("RZ", {"rz_plus": True}, {"linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                                   "angular": {"x": 0.0, "y": 0.0, "z": 1.0}}),
    ],
)
def test_mapeo_frame_por_grado_de_libertad(dof, kwargs, esperado):
    c = map_twist_components(**kwargs)
    for bloque in ("linear", "angular"):
        for eje, val in c[bloque].items():
            assert val == pytest.approx(esperado[bloque][eje])


def test_mapeo_plus_y_minus_a_la_vez_es_cero():
    c = map_twist_components(x_plus=True, x_minus=True)
    assert c["linear"]["x"] == pytest.approx(0.0)


def test_ganancia_multiplica_todos_los_componentes():
    c = map_twist_components(
        x_plus=True, y_minus=True, rz_plus=True, gain=2.0
    )
    assert c["linear"]["x"] == pytest.approx(2.0)
    assert c["linear"]["y"] == pytest.approx(-2.0)
    assert c["angular"]["z"] == pytest.approx(2.0)


def test_velocidad_escala_todos_los_componentes():
    c = map_twist_components(x_plus=True, y_plus=True, velocity_scale=0.5)
    assert c["linear"]["x"] == pytest.approx(0.5)
    assert c["linear"]["y"] == pytest.approx(0.5)
    assert c["angular"]["z"] == pytest.approx(0.0)


def test_velocidad_y_ganancia_son_independientes():
    c = map_twist_components(x_plus=True, gain=2.0)
    assert c["linear"]["x"] == pytest.approx(2.0)  # velocity_scale default 1.0
    c = map_twist_components(x_plus=True, velocity_scale=0.25)
    assert c["linear"]["x"] == pytest.approx(0.25)  # gain default 1.0
    c = map_twist_components(x_plus=True, gain=2.0, velocity_scale=0.25)
    assert c["linear"]["x"] == pytest.approx(0.5)  # producto de ambos


# ── Panel ───────────────────────────────────────────────────────────────

@pytest.fixture
def panel(qapp):
    return FrameControlPanel()


def test_panel_estado_inicial_cero(panel):
    assert panel.twist_components() == {
        "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
    }


def test_panel_botones_mapean_a_los_seis_dof(panel):
    panel.set_velocity_percent(100)
    casos = [
        ("X", "plus", "linear", "x", 1.0),
        ("X", "minus", "linear", "x", -1.0),
        ("Y", "plus", "linear", "y", 1.0),
        ("Z", "plus", "linear", "z", 1.0),
        ("RX", "plus", "angular", "x", 1.0),
        ("RY", "minus", "angular", "y", -1.0),
        ("RZ", "plus", "angular", "z", 1.0),
    ]
    for axis, sign, bloque, eje, esperado in casos:
        panel.reset()
        panel.set_button_pressed(axis, sign, True)
        comp = panel.twist_components()
        assert comp[bloque][eje] == pytest.approx(esperado), axis
        panel.set_button_pressed(axis, sign, False)
        assert panel.twist_components()[bloque][eje] == pytest.approx(0.0), axis


def test_panel_ganancia_aplica(panel):
    panel.set_velocity_percent(100)
    panel.set_button_pressed("X", "plus", True)
    panel.set_gain(2.00)
    assert panel.twist_components()["linear"]["x"] == pytest.approx(2.0)


def test_panel_reset_libera_todo(panel):
    panel.set_button_pressed("X", "plus", True)
    panel.set_button_pressed("RZ", "minus", True)
    assert panel.twist_components()["linear"]["x"] != 0.0
    panel.reset()
    assert panel.twist_components() == {
        "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
    }


def test_panel_readout_se_actualiza(panel):
    panel.set_velocity_percent(100)
    panel.set_button_pressed("RY", "plus", True)
    assert panel._readout["ang_y"].text() == "1.00"


def test_panel_velocidad_inicial_50_y_etiqueta(panel):
    assert panel.velocity_percent() == 50
    assert panel._vel_pct.text() == "50 %"
    assert panel._readout["lin_x"].text() == "0.00"


def test_panel_velocidad_escala_boton(panel):
    panel.set_button_pressed("X", "plus", True)
    assert panel.twist_components()["linear"]["x"] == pytest.approx(0.5)
    panel.set_velocity_percent(0)
    assert panel.twist_components()["linear"]["x"] == pytest.approx(0.0)
    panel.set_velocity_percent(100)
    assert panel.twist_components()["linear"]["x"] == pytest.approx(1.0)


def test_panel_velocidad_independiente_de_ganancia(panel):
    panel.set_button_pressed("X", "plus", True)
    panel.set_gain(2.00)
    panel.set_velocity_percent(25)
    assert panel.velocity_percent() == 25
    assert panel.gain() == pytest.approx(2.00)
    assert panel.twist_components()["linear"]["x"] == pytest.approx(0.5)


def test_panel_velocidad_actualiza_etiqueta_y_readout(panel):
    panel.set_button_pressed("Y", "plus", True)
    panel.set_velocity_percent(70)
    assert panel._vel_pct.text() == "70 %"
    assert panel._readout["lin_y"].text() == "0.70"  # 1.0 * 0.7


def test_panel_seguridad_plus_minus_simultaneos_es_cero(panel):
    panel.set_velocity_percent(100)
    panel.set_button_pressed("Z", "plus", True)
    panel.set_button_pressed("Z", "minus", True)
    assert panel.twist_components()["linear"]["z"] == pytest.approx(0.0)


# ── Integración con MainWindow (sin ROS) ────────────────────────────────

@pytest.fixture
def win(config_dir, qapp):
    from staubli_gui.gui import MainWindow

    window = MainWindow(ros_node=None)
    yield window
    window.close()


def test_ventana_crea_panel_y_no_publica_sin_ros(win):
    assert win._frame_panel is not None
    assert win._cmd_vel_pub is None
    assert not win._manual_active
    assert win._frame_panel.btn_toggle.text() == "ACTIVAR MANUAL"


def test_toggle_sin_ros_es_inocuo_y_registra_log(win):
    win._frame_panel.btn_toggle.setChecked(True)
    assert not win._manual_active  # sin ROS: no se publica nada
    assert "sin ROS" in win._log_edit.toPlainText()


def test_boton_refleja_estado_con_set_toggle_state(win):
    panel = win._frame_panel
    panel.set_toggle_state(True)
    assert panel.btn_toggle.isChecked()
    assert panel.btn_toggle.text() == "VOLVER A TRAYECTORIA"
    panel.set_toggle_state(False)
    assert not panel.btn_toggle.isChecked()
    assert panel.btn_toggle.text() == "ACTIVAR MANUAL"


def test_set_toggle_busy_bloquea_y_marca(win):
    panel = win._frame_panel
    panel.set_toggle_busy()
    assert not panel.btn_toggle.isEnabled()
    assert panel.btn_toggle.text() == "CAMBIO DE CONTROLADOR…"
    panel.set_toggle_state(False)  # restaura
    assert panel.btn_toggle.isEnabled()


def test_modo_actual_indicador(win):
    panel = win._frame_panel
    panel.set_mode_state(
        "trayectoria",
        controller="joint_trajectory_controller",
        velocity_pct=50,
    )
    assert panel._mode_label.text() == "MODO TRAYECTORIA"
    assert "joint_trajectory_controller" in panel._mode_info.text()
    panel.set_mode_state(
        "manual",
        controller="forward_velocity_controller",
        velocity_pct=50,
    )
    assert "MODO MANUAL" in panel._mode_label.text()
    assert "forward_velocity_controller" in panel._mode_info.text()
    panel.set_mode_state("conflicto")
    assert "CONFLICTO" in panel._mode_label.text()
    panel.set_mode_state("seguro")
    assert "MODO SEGURO" in panel._mode_label.text()
    panel.set_mode_state("desconocido")
    assert "DESCONOCIDO" in panel._mode_label.text()