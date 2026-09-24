"""Tests headless del feedback visual de puntos/trayectoria en la GUI.

Cubre la detección de cada waypoint "completado" mediante el feedback del
goal (FollowJointTrajectory), los mensajes de éxito/error del resultado y que
la trayectoria activa ese feedback por puntos. No se mueve ningún robot ni se
escribe en el home real (conftest.py aísla STAUBLI_GUI_CONFIG_DIR).
"""

import pytest
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

_PUNTOS_SEED = {
    "Punto P1": [0.1, -0.2, 0.3, -0.4, 0.5, -0.6],
    "Punto P2": [1.0, 1.1, 1.2, 1.3, 1.4, 1.5],
    "Punto P3": [-1.0, -1.1, -1.2, -1.3, -1.4, -1.5],
}
_Q1 = list(_PUNTOS_SEED["Punto P1"])
_Q2 = list(_PUNTOS_SEED["Punto P2"])
_Q3 = list(_PUNTOS_SEED["Punto P3"])


def _seed_user_points(config_dir):
    import yaml

    data = {"points": []}
    for nombre, q in _PUNTOS_SEED.items():
        data["points"].append(
            {
                "name": nombre,
                "position": [round(v * 0.1, 4) for v in q[:3]],
                "rpy_degrees": [0.0, 0.0, 0.0],
                "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                "q": list(q),
                "solved": True,
                "inside_limits": True,
            }
        )
    config_dir.mkdir(parents=True, exist_ok=True)
    (config_dir / "user_points.yaml").write_text(
        yaml.safe_dump(data, sort_keys=False), encoding="utf-8"
    )


@pytest.fixture
def win(config_dir, qapp):
    """Ventana de la GUI sin ROS, con Punto P1/P2/P3 en el almacén."""
    _seed_user_points(config_dir)
    from staubli_gui.gui import MainWindow

    window = MainWindow(ros_node=None)
    yield window
    window.close()


def _feedback_msg(desired, actual=None):
    """FeedbackMessage de action con desired.positions (y actual si se da).

    La GUI marca el punto como completado cuando feedback.desired coincide con
    el waypoint; 'actual' solo se usa en los tests que distinguen deseado vs
    medido (por defecto actual = desired).
    """
    fb = FollowJointTrajectory.Feedback()
    fb.desired = JointTrajectoryPoint()
    fb.desired.positions = [float(v) for v in desired]
    fb.actual = JointTrajectoryPoint()
    fb.actual.positions = [float(v) for v in (actual if actual is not None else desired)]

    class Msg:
        feedback = fb

    return Msg()


def _result_future(error_code, error_string=""):
    """Future falso cuyo result() devuelve el error_code JTC."""
    class Res:
        pass

    Res.error_code = error_code
    Res.error_string = error_string

    class Wrapped:
        result = Res()

    class Fut:
        def result(self):
            return Wrapped()

    return Fut()


def _log(win):
    return win._log_edit.toPlainText()


def _activo(win, button):
    win._motion_active = True
    win._motion_button = button
    button.setEnabled(False)


# ── Feedback: detección de puntos completados (solo visual) ─────────────


def test_feedback_marca_puntos_en_orden(win):
    win._motion_active = True
    win._wp_pending = [("Punto P1", _Q1), ("Punto P2", _Q2)]
    win._on_motion_feedback(_feedback_msg(_Q1))
    assert win._wp_pending == [("Punto P2", _Q2)]
    assert "Punto P1 completado" in _log(win)
    assert "Punto P2 completado" not in _log(win)


def test_feedback_salta_puntos_consecutivos_alcanzados(win):
    # Si el robot está en las q de varios puntos seguidos (p. ej. inicio de
    # trayectoria), se consumen todos los que coincidan en orden.
    win._motion_active = True
    win._wp_pending = [("Punto P1", _Q1), ("Punto P2", _Q1)]
    win._on_motion_feedback(_feedback_msg(_Q1))
    assert win._wp_pending == []
    assert "Punto P1 completado" in _log(win)
    assert "Punto P2 completado" in _log(win)


def test_feedback_respeta_el_orden(win):
    # El segundo punto coincide pero el primero no: no debe avanzar.
    win._motion_active = True
    win._wp_pending = [("Punto P1", _Q1), ("Punto P2", _Q2)]
    win._on_motion_feedback(_feedback_msg(_Q2))
    assert win._wp_pending == [("Punto P1", _Q1), ("Punto P2", _Q2)]
    assert "Punto P1 completado" not in _log(win)


def test_feedback_es_sensible_a_la_tolerancia(win):
    win._motion_active = True
    win._wp_pending = [("Punto P1", _Q1)]
    fuera = [v + 0.5 for v in _Q1]  # muy lejos del waypoint
    win._on_motion_feedback(_feedback_msg(fuera))
    assert win._wp_pending == [("Punto P1", _Q1)]

    dentro = [v + 0.01 for v in _Q1]  # dentro de +/- 0.02 rad
    win._on_motion_feedback(_feedback_msg(dentro))
    assert win._wp_pending == []


def test_feedback_ignorado_sin_movimiento_activo(win):
    win._wp_pending = [("Punto P1", _Q1)]
    win._motion_active = False
    win._on_motion_feedback(_feedback_msg(_Q1))
    assert win._wp_pending == [("Punto P1", _Q1)]


def test_feedback_ultimo_punto_refleja_estado(win):
    win._motion_active = True
    win._wp_pending = [("Punto P1", _Q1)]
    win._on_motion_feedback(_feedback_msg(_Q1))
    assert "último punto" in win._status_label.text()


def test_feedback_usa_desired_aunque_actual_no_coincida(win):
    # El punto se considera completado cuando desired coincide, aunque la
    # posición medida (actual) todavía esté lejos del waypoint (caso real
    # observado en ejecución: desired pasa a <0.005 rad, actual fallaba).
    win._motion_active = True
    win._wp_pending = [("Punto P1", _Q1)]
    actual_lejos = [v + 0.9 for v in _Q1]
    win._on_motion_feedback(_feedback_msg(_Q1, actual=actual_lejos))
    assert win._wp_pending == []
    assert "Punto P1 completado" in _log(win)
    assert "último punto" in win._status_label.text()


def test_feedback_secuencia_p1_p2_p3(win):
    # desired alcanza P1, luego P2 y luego P3: se muestran los tres, en orden.
    win._motion_active = True
    win._wp_pending = [("Punto P1", _Q1), ("Punto P2", _Q2), ("Punto P3", _Q3)]
    win._on_motion_feedback(_feedback_msg(_Q1))
    win._on_motion_feedback(_feedback_msg(_Q2))
    win._on_motion_feedback(_feedback_msg(_Q3))
    assert win._wp_pending == []
    log = _log(win)
    orden = [log.find(t) for t in ["Punto P1 completado", "Punto P2 completado", "Punto P3 completado"]]
    assert -1 not in orden
    assert orden[0] < orden[1] < orden[2]


def test_feedback_no_genera_mensajes_duplicados(win):
    win._motion_active = True
    win._wp_pending = [("Punto P1", _Q1), ("Punto P2", _Q2)]
    win._on_motion_feedback(_feedback_msg(_Q1))
    # El mismo feedback de P1 vuelve a llegar (mas de una muestra del JTC)
    # mientras el robot aún está cerca: no debe repetirse "P1 completado".
    win._on_motion_feedback(_feedback_msg([v + 0.01 for v in _Q1]))
    win._on_motion_feedback(_feedback_msg(_Q1))
    assert _log(win).count("Punto P1 completado") == 1
    assert win._wp_pending == [("Punto P2", _Q2)]


def test_secuencia_completa_puntos_y_final(win):
    # Al terminar, además de los "✓ <punto> completado" intermedios, sigue
    # apareciendo el mensaje final "✓ Trayectoria completada".
    _activo(win, win.btn_run)
    win._wp_pending = [("Punto P1", _Q1), ("Punto P2", _Q2), ("Punto P3", _Q3)]
    win._on_motion_feedback(_feedback_msg(_Q1))
    win._on_motion_feedback(_feedback_msg(_Q2))
    win._on_motion_feedback(_feedback_msg(_Q3))
    assert win._wp_pending == []
    win._on_motion_result(
        _result_future(FollowJointTrajectory.Result.SUCCESSFUL),
        ok_text="Trayectoria completada",
        tag="TRAYECTORIA",
    )
    log = _log(win)
    assert "Punto P3 completado" in log
    assert "Trayectoria completada" in log


# ── Resultado del goal: éxito/error por punto ───────────────────────────


def test_resultado_ok_muestra_el_punto_completado(win):
    _activo(win, win.btn_p1)
    win._wp_pending = None
    win._on_motion_result(
        _result_future(FollowJointTrajectory.Result.SUCCESSFUL),
        ok_text="P1 completado",
        tag="P1",
    )
    assert "P1 completado" in _log(win)
    assert win._motion_active is False
    assert win.btn_p1.isEnabled()


def test_resultado_trayectoria_ok_completa(win):
    _activo(win, win.btn_run)
    win._wp_pending = None
    win._on_motion_result(
        _result_future(FollowJointTrajectory.Result.SUCCESSFUL),
        ok_text="Trayectoria completada",
        tag="TRAYECTORIA",
    )
    assert "Trayectoria completada" in _log(win)


def test_resultado_fallo_indica_el_punto_no_alcanzado(win):
    _activo(win, win.btn_run)
    win._wp_pending = [("Punto P3", _Q3), ("Punto P1", _Q1)]
    win._on_motion_result(
        _result_future(FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED),
        ok_text="Trayectoria completada",
        tag="TRAYECTORIA",
        fail_text="TRAYECTORIA: MOVIMIENTO FALLIDO",
    )
    assert "Punto P3: FALLÓ" in _log(win)
    assert win._wp_pending is None


def test_resultado_fallo_sin_puntos_usa_el_tag(win):
    _activo(win, win.btn_p1)
    win._wp_pending = None
    win._on_motion_result(
        _result_future(FollowJointTrajectory.Result.INVALID_GOAL),
        ok_text="P1 completado",
        tag="P1",
    )
    assert "P1: FALLÓ" in _log(win)


def test_resultado_cancelado_por_stop_no_marca_punto(win):
    _activo(win, win.btn_run)
    win._wp_pending = [("Punto P1", _Q1)]
    win._stop_requested = True
    win._on_motion_result(
        _result_future(FollowJointTrajectory.Result.SUCCESSFUL),
        ok_text="Trayectoria completada",
        tag="TRAYECTORIA",
    )
    assert "Trayectoria completada" not in _log(win)
    assert "goal cancelado por STOP" in _log(win)
    assert win._status_label.text() == "● MOVIMIENTO DETENIDO"
    assert win._wp_pending is None


# ── La trayectoria activa el feedback por puntos ────────────────────────


def test_on_run_envia_waypoints_de_feedback(win, monkeypatch):
    win._user_trajectory = ["Punto P1", "Punto P3"]
    win._ros_node = object()
    win._action_client = object()
    win._ctrl = object()
    capturados = {}
    monkeypatch.setattr(win, "_run_goal", lambda **kw: capturados.update(kw))
    win._on_run()
    assert [n for n, _ in capturados["waypoints"]] == ["Punto P1", "Punto P3"]
    assert capturados["waypoints"][0][1] == pytest.approx(_Q1)
    assert capturados["waypoints"][1][1] == pytest.approx(_Q3)
    assert capturados["ok_text"] == "Trayectoria completada"


def test_run_goal_activa_el_feedback_cuando_hay_waypoints(win, monkeypatch):
    from unittest import mock  # noqa: PLC0415

    class FakeLogger:
        def info(self, *args, **kwargs):
            pass

    class FakeNode:
        def get_logger(self):
            return FakeLogger()

    class FakeActionClient:
        def __init__(self):
            self.captured = {}

        def server_is_ready(self):
            return True

        def send_goal_async(self, goal, feedback_callback=None):
            self.captured["cb"] = feedback_callback
            fut = mock.MagicMock()
            fut.add_done_callback = lambda fn: None
            return fut

    win._ros_node = FakeNode()
    win._action_client = FakeActionClient()
    win._ctrl = object()  # _switch_to_mode será sustituido y no lo usa
    monkeypatch.setattr(
        win, "_switch_to_mode", lambda mode, after=None, **kw: after(True, "ok")
    )
    goal = win._build_single_goal(_Q1, 5.0)
    win._run_goal(
        goal=goal,
        button=win.btn_run,
        moving_text="X",
        ok_text="Trayectoria completada",
        tag="TRAYECTORIA",
        waypoints=[("Punto P1", _Q1)],
    )
    assert win._wp_pending == [("Punto P1", _Q1)]
    assert win._action_client.captured["cb"] is not None
    # Limpieza del estado dejado por el flujo simulado sin callback de resultado.
    win._motion_active = False
    win._wp_pending = None
    win._motion_button = None

    win._run_goal(
        goal=win._build_single_goal(_Q1, 5.0),
        button=win.btn_run,
        moving_text="X",
        ok_text="P1 completado",
        tag="P1",
    )
    assert win._wp_pending is None
    assert win._action_client.captured["cb"] is None