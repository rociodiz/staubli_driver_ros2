"""Tests headless de la "trayectoria definida por el usuario".

No se mueve ningún robot: se contrastan la lista interna
(_user_trajectory), el estado de los botones y el goal construido
(_build_user_trajectory_goal). Escribir aquí
(~/.config/staubli_gui/user_points.yaml real) está prohibido por diseño:
siempre se usa STAUBLI_GUI_CONFIG_DIR (ver conftest.py).
"""

import pytest
import yaml

_PUNTOS_SEED = {
    "Punto P1": [0.1, -0.2, 0.3, -0.4, 0.5, -0.6],
    "Punto P2": [1.0, 1.1, 1.2, 1.3, 1.4, 1.5],
    "Punto P3": [-1.0, -1.1, -1.2, -1.3, -1.4, -1.5],
}


def _seed_user_points(config_dir):
    """Crea un user_points.yaml de prueba en el directorio aislado."""
    data = {"points": []}
    for nombre, q in _PUNTOS_SEED.items():
        data["points"].append(
            {
                "name": nombre,
                "position": [round(v * 0.1, 4) for v in q[:3]],
                "rpy_degrees": list(q[3:]),
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
    """Ventana de la GUI sin ROS (layout/widgets reales, spin desactivado)."""
    _seed_user_points(config_dir)
    from staubli_gui.gui import MainWindow

    window = MainWindow(ros_node=None)
    yield window
    window.close()


# ── Helpers ─────────────────────────────────────────────────────────────


def _qa(nombre):
    """Devuelve la q esperada de un punto definido en _PUNTOS_SEED."""
    return _PUNTOS_SEED[nombre]


def _seleccionar_punto_usuario(win, nombre):
    textos = [
        win._points_list.item(i).text()
        for i in range(win._points_list.count())
    ]
    win._points_list.setCurrentRow(textos.index(nombre))


def _seleccionar_fila_trayectoria(win, row):
    win._trajectory_list.setCurrentRow(row)


# ── Tests ───────────────────────────────────────────────────────────────


def test_estado_inicial(win):
    assert win._user_trajectory == []
    assert win._trajectory_list.count() == 0
    assert not win.btn_add_to_traj.isEnabled()
    assert not win.btn_traj_up.isEnabled()
    assert not win.btn_traj_down.isEnabled()
    assert not win.btn_traj_remove.isEnabled()
    assert not win.btn_traj_clear.isEnabled()


def test_añadir_preserva_orden_y_permite_duplicados(win, config_dir):
    _seleccionar_punto_usuario(win, "Punto P1")
    win._on_add_to_trajectory()
    _seleccionar_punto_usuario(win, "Punto P3")
    win._on_add_to_trajectory()
    _seleccionar_punto_usuario(win, "Punto P2")
    win._on_add_to_trajectory()
    assert win._user_trajectory == ["Punto P1", "Punto P3", "Punto P2"]
    assert win._trajectory_list.count() == 3
    assert win._trajectory_list.item(0).text() == "1. Punto P1"
    assert win._trajectory_list.item(1).text() == "2. Punto P3"
    assert win._trajectory_list.item(2).text() == "3. Punto P2"

    # Duplicados permitidos: una trayectoria puede repetir puntos.
    _seleccionar_punto_usuario(win, "Punto P1")
    win._on_add_to_trajectory()
    assert win._user_trajectory == [
        "Punto P1", "Punto P3", "Punto P2", "Punto P1",
    ]


def test_añadir_sin_seleccion_no_hace_nada(win):
    win._on_add_to_trajectory()
    assert win._user_trajectory == []


def test_subir_y_bajar_elemento(win):
    win._user_trajectory = ["Punto P1", "Punto P3", "Punto P2"]
    win._refresh_trajectory_list()
    _seleccionar_fila_trayectoria(win, 2)  # Punto P2
    win._on_traj_move_up()
    assert win._user_trajectory == ["Punto P1", "Punto P2", "Punto P3"]
    _seleccionar_fila_trayectoria(win, 2)  # Punto P3
    win._on_traj_move_down()  # ya es el último: no-op
    assert win._user_trajectory == ["Punto P1", "Punto P2", "Punto P3"]
    _seleccionar_fila_trayectoria(win, 0)
    win._on_traj_move_up()  # ya es el primero: no-op
    assert win._user_trajectory == ["Punto P1", "Punto P2", "Punto P3"]


def test_quitar_no_borra_el_punto(win):
    win._user_trajectory = ["Punto P1", "Punto P3", "Punto P2"]
    win._refresh_trajectory_list()
    _seleccionar_fila_trayectoria(win, 1)  # Punto P3
    win._on_traj_remove()
    assert win._user_trajectory == ["Punto P1", "Punto P2"]
    assert win._store.has("Punto P3")  # intacto en Puntos del usuario


def test_limpiar_solo_vacia_la_trayectoria(win):
    win._user_trajectory = ["Punto P1", "Punto P3", "Punto P2"]
    win._refresh_trajectory_list()
    win._on_traj_clear()
    assert win._user_trajectory == []
    assert win._trajectory_list.count() == 0
    assert sorted(win._store.names()) == ["Punto P1", "Punto P2", "Punto P3"]


def test_ejecutar_con_lista_vacia_no_envia_goal(win, monkeypatch):
    enviados = []
    win._ros_node = object()
    win._action_client = object()
    win._list_controllers_client = object()
    monkeypatch.setattr(win, "_run_goal", lambda **kw: enviados.append(kw))
    win._on_run()
    assert enviados == []  # ningún goal
    assert "no hay puntos definidos" in win._log_edit.toPlainText()


def test_goal_posiciones_y_tiempos(win):
    win._user_trajectory = ["Punto P1", "Punto P3", "Punto P2"]
    goal = win._build_user_trajectory_goal()
    traj = goal.trajectory
    assert traj.joint_names == [f"joint_{i}" for i in range(1, 7)]
    assert len(traj.points) == 3
    for i, nombre in enumerate(["Punto P1", "Punto P3", "Punto P2"]):
        p = traj.points[i]
        assert list(p.positions) == _qa(nombre)
        t = p.time_from_start.sec + p.time_from_start.nanosec / 1e9
        assert t == pytest.approx(5.0 * (i + 1))  # 5, 10, 15...
    assert len(goal.goal_tolerance) == 6
    assert goal.goal_time_tolerance.sec == 2  # 2.5 s (tolerancia existente)


def test_los_userpoints_originales_quedan_intactos(win):
    antes = {n: list(win._store.get(n).q) for n in win._store.names()}
    win._user_trajectory = ["Punto P1", "Punto P3", "Punto P2"]
    win._refresh_trajectory_list()
    _seleccionar_fila_trayectoria(win, 1)
    win._on_traj_remove()
    win._on_traj_clear()
    despues = {n: list(win._store.get(n).q) for n in win._store.names()}
    assert antes == despues