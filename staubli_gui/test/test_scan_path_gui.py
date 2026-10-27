"""Tests headless del panel de barrido integrado en la GUI (fase 2).

No se mueve ningún robot: se comprueba que la GUI
  1. traduce sus controles (mm) a la fuente de verdad de scan_path (metros),
  2. delega TODO el cálculo (geometría, orientación, IK, límites) en
     scan_path.resolve_scan, sin duplicarlo,
  3. pinta la tabla P1/P2 con el veredicto real,
  4. guarda y reabre el YAML persistiendo solo la fuente de verdad.

Nada de esto toca trajectory_builder ni el JTC (fase posterior).
"""

import math

import pytest
import yaml

from staubli_gui import gui as gui_mod
from staubli_gui.scan_path import (
    DEFAULT_PART_CENTER,
    FACE_NORMALS,
    ROLES,
    ScanPathSpec,
    ScanPathStore,
    ScanProfile,
    resolve_scan,
)

DIST_INICIAL_MM = 325
LARGO_INICIAL_MM = 80

# Columnas de la tabla: 0 rol, 1-3 xyz, 4 rpy, 5 distancia, 6 ik, 7 límites,
# 8 articulación culpable.
COL_ROL, COL_X, COL_Y, COL_Z, COL_RPY = 0, 1, 2, 3, 4
COL_DIST, COL_IK, COL_LIMITES, COL_CULPABLE = 5, 6, 7, 8


@pytest.fixture
def ventana(qapp, config_dir):
    """MainWindow con la configuración aislada y sin nodo ROS."""
    ventana = gui_mod.MainWindow(ros_node=None)
    return ventana


def _texto(ventana, row, col):
    return ventana._scan_table.item(row, col).text()


def _fila(ventana, row):
    return [ventana._scan_table.item(row, c).text() for c in range(9)]


def _regenerar(
    ventana,
    face=None,
    dist_mm=None,
    largo_mm=None,
    margen_start_mm=None,
    margen_end_mm=None,
):
    """Ajusta los controles y pulsa REGENERAR (como haría el usuario)."""
    if face is not None:
        ventana._scan_face_combo.setCurrentText(face)
    if dist_mm is not None:
        ventana._scan_distance_spin.setValue(dist_mm)
    if largo_mm is not None:
        ventana._scan_length_spin.setValue(largo_mm)
    if margen_start_mm is not None:
        ventana._scan_margin_start_spin.setValue(margen_start_mm)
    if margen_end_mm is not None:
        ventana._scan_margin_end_spin.setValue(margen_end_mm)
    ventana.btn_scan_regenerate.click()
    return ventana._scan_solution


# ── Controles: valores iniciales y unidades ────────────────────────────────


def test_valores_iniciales_de_los_controles(ventana):
    assert ventana._scan_face_combo.currentText() == "+Z"
    assert ventana._scan_distance_spin.value() == DIST_INICIAL_MM
    assert ventana._scan_length_spin.value() == LARGO_INICIAL_MM
    assert ventana._scan_distance_spin.suffix() == " mm"


def test_las_seis_caras_estan_disponibles(ventana):
    disponibles = [
        ventana._scan_face_combo.itemText(i)
        for i in range(ventana._scan_face_combo.count())
    ]
    assert disponibles == ["+Z", "-Z", "+X", "-X", "+Y", "-Y"]
    assert set(disponibles) == set(FACE_NORMALS)


def test_la_distancia_admite_de_100_a_500_mm(ventana):
    spin = ventana._scan_distance_spin
    assert spin.minimum() <= 100
    assert spin.maximum() >= 500
    for mm in (100, 200, 300, 400, 500):
        spin.setValue(mm)
        assert spin.value() == mm


def test_cambio_de_milimetros_a_metros_usa_el_factor_correcto():
    assert gui_mod._spin_to_m(300) == pytest.approx(0.30)
    assert gui_mod._spin_to_m(80) == pytest.approx(0.08)
    assert gui_mod._m_to_mm(0.30) == pytest.approx(300.0)
    assert gui_mod._mm_to_spin(0.30) == 300
    assert gui_mod._mm_to_spin(0.08) == 80


def test_el_controlador_marca_la_pieza_por_defecto_correcta(ventana):
    """La GUI no inventa la pieza: la toma del almacén (source of truth)."""
    spec = ventana._scan_current_spec()
    perfil = ventana._scan_current_profile()
    assert perfil.part.center == pytest.approx(DEFAULT_PART_CENTER)
    assert spec.face == "+Z"
    assert spec.line_length == pytest.approx(0.08)


# ── REGENERAR delega todo en scan_path ─────────────────────────────────────


def test_regenerar_produce_la_solucion_de_scan_path(ventana):
    """La GUI no calcula nada: refleja el ScanSolution de scan_path."""
    solution = _regenerar(ventana, margen_start_mm=0, margen_end_mm=0)
    referencia = resolve_scan(
        ScanProfile(),
        ScanPathSpec(name="superior_h", face="+Z", line_axis=[0.0, 1.0, 0.0]),
        None,
    )
    assert solution is not None
    assert solution.spec.face == referencia.spec.face
    for point, esperado in zip(solution.points, referencia.points):
        assert point.position_xyz == pytest.approx(esperado.position_xyz, abs=1e-12)
        assert point.rpy_degrees == pytest.approx(esperado.rpy_degrees, abs=1e-9)
        assert point.solved == esperado.solved
        assert point.inside_limits == esperado.inside_limits


def test_la_gui_no_duplicate_calculos_de_geometria(ventana):
    """No hay coordenadas escritas a mano en gui.py: salen de scan_path."""
    solution = _regenerar(ventana, margen_start_mm=0, margen_end_mm=0)
    esperado = ScanProfile().target_positions(
        ScanPathSpec(name="x", face="+Z", line_axis=[0.0, 1.0, 0.0])
    )
    for point, objetivo in zip(solution.points, esperado):
        assert point.position_xyz == pytest.approx(objetivo, abs=1e-12)


def test_posiciones_iniciales_validadas_desde_la_gui(ventana):
    """+Z a 325 mm de la mesa, de -Y a +Y: P1 y P2 sobre z=0.20+0.325."""
    _regenerar(ventana, margen_start_mm=0, margen_end_mm=0)
    # P1 = [0, 0.42, 0.525]
    assert _texto(ventana, 0, COL_X) == "+0.000"
    assert _texto(ventana, 0, COL_Y) == "+0.420"
    assert _texto(ventana, 0, COL_Z) == "+0.525"
    # P2 = [0, 0.50, 0.525]
    assert _texto(ventana, 1, COL_X) == "+0.000"
    assert _texto(ventana, 1, COL_Y) == "+0.500"
    assert _texto(ventana, 1, COL_Z) == "+0.525"


def test_la_tabla_muestra_los_dos_puntos_con_nombre(ventana):
    _regenerar(ventana)
    assert [_texto(ventana, r, COL_ROL) for r in range(2)] == list(ROLES)
    # No hay tercera fila de barrido: el recorrido termina en P2.
    assert ventana._scan_table.rowCount() == 2


def test_la_tabla_muestra_orientacion_y_distancia(ventana):
    solution = _regenerar(ventana)
    for row, point in enumerate(solution.points):
        assert "," in _texto(ventana, row, COL_RPY)
        assert _texto(ventana, row, COL_DIST) != "—"


def test_configuracion_valida_muestra_verde_y_todo_correcto(ventana):
    _regenerar(ventana)
    assert ventana._scan_verdict.text().startswith("✓")
    assert "VÁLIDO" in ventana._scan_verdict.text()
    for row in range(2):
        assert _texto(ventana, row, COL_IK) == "✓"
        assert _texto(ventana, row, COL_LIMITES) == "✓"
        assert _texto(ventana, row, COL_CULPABLE) == "—"
    assert "RETIRADA" not in ventana._scan_verdict.text()


def test_la_fila_se_pinta_de_verde_cuando_el_punto_es_valido(ventana):
    _regenerar(ventana)
    verde = ventana._scan_table.item(0, COL_IK).background().color().name()
    assert verde == gui_mod._VERDE


def test_las_distancias_de_trabajo_son_validas_para_esta_pieza(ventana):
    """+Z es alcanzable de 200 a 500 mm de la mesa: nada de un tope supuesto."""
    for mm in (200, 300, 400, 500):
        _regenerar(ventana, dist_mm=mm)
        assert ventana._scan_solution.valid, f"{mm} mm debería ser válido"
        assert ventana._scan_verdict.text().startswith("✓")


# ── Validación real (nada hardcodeado) ────────────────────────────────────


def test_una_cara_no_alcanzable_se_muestra_invalida(ventana):
    """-Z (sobre la mesa) no resuelve: la GUI lo dice, no lo descarta."""
    _regenerar(ventana, face="-Z")
    assert not ventana._scan_solution.valid
    assert ventana._scan_verdict.text().startswith("✗")
    assert "NO VÁLIDO" in ventana._scan_verdict.text()
    assert _texto(ventana, 0, COL_IK) == "✗"


def test_la_celda_culpable_coincide_con_scan_path(ventana):
    """La articulación culpable la decide scan_path, no la GUI."""
    for face in FACE_NORMALS:
        _regenerar(ventana, face=face)
        solution = ventana._scan_solution
        for row, point in enumerate(solution.points):
            esperado = ", ".join(point.offending_joint_names()) or "—"
            assert _texto(ventana, row, COL_CULPABLE) == esperado
            assert _texto(ventana, row, COL_IK) == ("✓" if point.solved else "✗")
            assert _texto(ventana, row, COL_LIMITES) == (
                "✓" if point.inside_limits else "✗"
            )


def test_la_fila_se_pinta_de_rojo_cuando_el_punto_no_es_valido(ventana):
    _regenerar(ventana, face="-Z")
    rojo = ventana._scan_table.item(0, COL_IK).background().color().name()
    assert rojo == gui_mod._ROJO


def test_un_limite_articular_se_nombra_en_la_tabla(ventana):
    """-X a 350 mm sin inclinar: P1 resuelve pero joint_5 se sale de rango."""
    ventana._scan_store.profile.max_tilt_deg = 0.0
    _regenerar(ventana, face="-X", dist_mm=350)
    solution = ventana._scan_solution
    assert solution.offending_joints, "se esperaba joint_5 fuera de rango"
    assert not solution.valid
    culpable = ", ".join(f"joint_{j}" for j in solution.offending_joints)
    assert culpable in " ".join(
        _texto(ventana, r, COL_CULPABLE) for r in range(2)
    )


# ── Dirección del barrido ─────────────────────────────────────────────────


def test_la_direccion_por_defecto_se_adapta_a_la_cara(ventana):
    ventana._scan_face_combo.setCurrentText("+Z")
    assert tuple(ventana._scan_axis_combo.currentData()) == (0.0, 1.0, 0.0)
    ventana._scan_face_combo.setCurrentText("+X")
    assert tuple(ventana._scan_axis_combo.currentData()) == (0.0, 1.0, 0.0)


def test_el_boton_defecto_restaura_la_direccion_de_la_cara(ventana):
    ventana._scan_face_combo.setCurrentText("+X")
    ventana._scan_axis_combo.setCurrentIndex(0)  # eje de otra cara
    ventana._scan_axis_default_btn.click()
    assert tuple(ventana._scan_axis_combo.currentData()) == (0.0, 1.0, 0.0)


def test_la_direccion_se_guarda_y_se_reabre(ventana):
    ventana._scan_name_edit.setText("frontal_x")
    ventana._scan_face_combo.setCurrentText("+X")
    ventana._on_scan_save()
    reabierta = gui_mod.MainWindow(ros_node=None)
    assert tuple(reabierta._scan_axis_combo.currentData()) == (0.0, 1.0, 0.0)
    assert reabierta._scan_name_edit.text() == "frontal_x"


def test_la_direccion_proyectada_es_tangente_a_la_cara(ventana):
    """La geometría de scan_path proyecta el eje sobre el plano de la cara."""
    _regenerar(ventana, face="+Z")
    eje = ventana._scan_solution.profile.tangent_axis(ventana._scan_solution.spec)
    normal = ventana._scan_solution.profile.part.face_normal("+Z")
    assert sum(a * b for a, b in zip(eje, normal)) == pytest.approx(0.0, abs=1e-12)


def test_una_direccion_imposible_no_rompe_la_gui(ventana):
    """Un eje paralelo a la normal se informa, no revienta la ventana.

    El error lo detecta scan_path al proyectar el eje sobre el plano de la
    cara (dentro de resolve_scan), así que la GUI lo muestra como fallo de
    cálculo y conserva el motivo exacto.
    """
    ventana._scan_face_combo.setCurrentText("+Z")
    ventana._scan_axis_combo.addItem("[custom]", (0.0, 0.0, 1.0))
    ventana._scan_axis_combo.setCurrentIndex(ventana._scan_axis_combo.count() - 1)
    ventana.btn_scan_regenerate.click()
    assert ventana._scan_verdict.text().startswith("✗")
    assert "line_axis es paralelo a la normal" in ventana._scan_verdict.text()


# ── GUARDAR: solo fuente de verdad ────────────────────────────────────────


def test_guardar_escribe_el_esquema_de_scan_path(ventana):
    ventana.btn_scan_save.click()
    data = yaml.safe_load(ventana._scan_store.path.read_text(encoding="utf-8"))
    assert set(data) == {
        "version",
        "part",
        "scan_distance",
        "max_tilt_deg",
        "scan_paths",
    }
    assert data["version"] == 3
    assert data["scan_distance"] == pytest.approx(0.325)
    assert data["max_tilt_deg"] == pytest.approx(80.0)
    spec = data["scan_paths"][0]
    assert spec["face"] == "+Z"
    assert spec["line_length"] == pytest.approx(0.08)
    assert spec["line_axis"] == pytest.approx([0.0, 1.0, 0.0])


def test_guardar_no_persiste_puntos_ni_validacion(ventana):
    """P1/P2 y validación siguen siendo datos derivados."""
    _regenerar(ventana)
    ventana.btn_scan_save.click()
    texto = ventana._scan_store.path.read_text(encoding="utf-8")
    for prohibido in ("points", "rpy_degrees", "quaternion", "validation", "solved"):
        assert prohibido not in texto


def test_guardar_refleja_la_configuracion_actual(ventana):
    _regenerar(ventana, dist_mm=400, largo_mm=120)
    ventana.btn_scan_save.click()
    data = yaml.safe_load(ventana._scan_store.path.read_text(encoding="utf-8"))
    assert data["scan_distance"] == pytest.approx(0.40)
    assert "retract" not in data
    assert data["scan_paths"][0]["line_length"] == pytest.approx(0.12)


def test_guardar_no_persiste_la_ik_ni_los_limites(ventana):
    """Ni q, ni validado, ni dist_measured llegan al fichero."""
    _regenerar(ventana)
    ventana.btn_scan_save.click()
    texto = ventana._scan_store.path.read_text(encoding="utf-8")
    for prohibido in ("q:", "joint_", "inside_limits", "offending", "measured"):
        assert prohibido not in texto


# ── REABRIR: reconstrucción desde el YAML ─────────────────────────────────


def test_reabrir_reconstruye_los_puntos_desde_el_yaml(ventana):
    _regenerar(ventana, margen_start_mm=0, margen_end_mm=0)
    ventana.btn_scan_save.click()
    reabierta = gui_mod.MainWindow(ros_node=None)
    assert reabierta._scan_face_combo.currentText() == "+Z"
    assert reabierta._scan_distance_spin.value() == DIST_INICIAL_MM
    assert reabierta._scan_length_spin.value() == LARGO_INICIAL_MM
    assert _texto(reabierta, 0, COL_Y) == "+0.420"
    assert _texto(reabierta, 0, COL_Z) == "+0.525"
    assert _texto(reabierta, 1, COL_Y) == "+0.500"
    assert _texto(reabierta, 1, COL_Z) == "+0.525"
    assert reabierta._scan_verdict.text().startswith("✓")


def test_reabrir_con_una_config_no_alcanzable_la_muestra_invalida(ventana):
    """Requisito: config inalcanzable visible, nunca descartada en silencio."""
    _regenerar(ventana, face="-Z")
    ventana.btn_scan_save.click()
    reabierta = gui_mod.MainWindow(ros_node=None)
    assert reabierta._scan_face_combo.currentText() == "-Z"
    assert reabierta._scan_solution is not None
    assert not reabierta._scan_solution.valid
    assert reabierta._scan_verdict.text().startswith("✗")


def test_reabrir_sin_yaml_no_falla(ventana):
    limpia = gui_mod.MainWindow(ros_node=None)
    assert limpia._scan_solution is None
    assert "Sin barrido guardado" in limpia._scan_verdict.text()
    assert _texto(limpia, 0, COL_ROL) == "—"


def test_reabrir_con_yaml_corrupto_no_falla(ventana, config_dir):
    (config_dir / "scan_paths.yaml").write_text("no: [esto", encoding="utf-8")
    rota = gui_mod.MainWindow(ros_node=None)
    assert rota._scan_verdict.text() != ""
    assert not rota._scan_verdict.text().startswith("✓")


def test_cambiar_la_distancia_no_deja_puntos_residuales(ventana):
    """La altura sigue a la distancia medida desde la mesa (z=0.20+d)."""
    _regenerar(ventana, dist_mm=500)
    assert _texto(ventana, 0, COL_Z) == "+0.700"
    _regenerar(ventana, dist_mm=200)
    assert _texto(ventana, 0, COL_Z) == "+0.400"


def test_la_distancia_medida_coincide_con_la_pedida(ventana):
    """La columna 'Dist.' viene del FK de scan_path, no de restar números."""
    _regenerar(ventana, dist_mm=400)
    solution = ventana._scan_solution
    for row, point in enumerate(solution.points):
        mostrado = float(_texto(ventana, row, COL_DIST))
        esperado_mm = point.distance_measured * gui_mod._SCAN_MM
        assert mostrado == pytest.approx(esperado_mm, abs=0.05)
        # P1 y P2 están ambos a la distancia pedida: no hay punto retreated.
        assert mostrado == pytest.approx(400.0, abs=0.05)
    assert solution.point("P3") is None


def test_el_escaneo_mide_la_linea_completa(ventana):
    _regenerar(ventana, largo_mm=80, margen_start_mm=0, margen_end_mm=0)
    solution = ventana._scan_solution
    assert solution.scan_length == pytest.approx(0.08, abs=1e-12)
    largo_real = math.dist(
        solution.point("P1").position_xyz, solution.point("P2").position_xyz
    )
    assert largo_real == pytest.approx(0.08, abs=1e-12)


def test_la_longitud_configurada_no_mueve_los_extremos(ventana):
    """'Longitud barrido' es nominal: P1/P2 solo dependen de los bordes reales."""
    _regenerar(ventana, largo_mm=80, margen_start_mm=5, margen_end_mm=20)
    y_80 = (_texto(ventana, 0, COL_Y), _texto(ventana, 1, COL_Y))
    for largo in (10, 40, 120, 300):
        _regenerar(ventana, largo_mm=largo, margen_start_mm=5, margen_end_mm=20)
        assert (_texto(ventana, 0, COL_Y), _texto(ventana, 1, COL_Y)) == y_80
    # Los extremos son los bordes reales (0.42/0.50) más el margen de cada lado.
    assert y_80 == ("+0.415", "+0.520")


# ── Aislamiento de la fase 2 ──────────────────────────────────────────────


def test_el_panel_no_toca_trajectory_builder(ventana, monkeypatch):
    """Ninguna acción del panel debe construir JointTrajectory ni goals."""
    import staubli_gui.gui as g

    assert "trajectory_builder" not in g.__dict__
    for attr in ("btn_run", "btn_stop", "btn_move_point"):
        # Los botones de movimiento siguen siendo los del JTC, sin cambios.
        assert hasattr(ventana, attr)


def test_ningun_punto_derivado_se_anade_a_la_trayectoria_del_usuario(ventana):
    """REGENERAR no toca la trayectoria ni el almacén de puntos del usuario."""
    nombres_antes = list(ventana._store.names())
    _regenerar(ventana)
    assert list(ventana._store.names()) == nombres_antes
    assert ventana._trajectory_list.count() == 0


def test_regenerar_repetido_es_estable(ventana):
    """Misma configuración -> mismo resultado (cinemática nueva por ciclo)."""
    primera = _regenerar(ventana)
    puntos_1 = [list(p.position_xyz) for p in primera.points]
    validos_1 = [p.valid for p in primera.points]
    segunda = _regenerar(ventana)
    assert [list(p.position_xyz) for p in segunda.points] == puntos_1
    assert [p.valid for p in segunda.points] == validos_1


def test_el_panel_usa_el_almacen_de_scan_path(ventana):
    assert isinstance(ventana._scan_store, ScanPathStore)
    assert ventana._scan_store.path.name == "scan_paths.yaml"


def test_varias_configuraciones_conviven_en_el_almacen(ventana, config_dir):
    ventana._scan_name_edit.setText("superior_h")
    ventana.btn_scan_save.click()
    ventana._scan_name_edit.setText("frontal_x")
    ventana._scan_face_combo.setCurrentText("+X")
    ventana.btn_scan_save.click()
    data = yaml.safe_load(ventana._scan_store.path.read_text(encoding="utf-8"))
    assert [s["name"] for s in data["scan_paths"]] == ["superior_h", "frontal_x"]
    assert {s["face"] for s in data["scan_paths"]} == {"+Z", "+X"}


# ── Márgenes asimétricos desde la GUI ────────────────────────────────────
#
# Con el sentido por defecto de +Z (line_axis = +Y) P1 arranca en el borde
# cercano (-Y) y P2 llega al borde opuesto (+Y). El margen de inicio retrasa
# P1 hacia FUERA (-Y) y el de fin avanza P2 hacia FUERA (+Y). Dos controles
# independientes: sus defaults NO son iguales.


def test_los_margenes_por_defecto_son_asimetricos(ventana):
    """5 mm al inicio y 20 mm al final, según manda cada extremo."""
    assert ventana._scan_margin_start_spin.value() == 5
    assert ventana._scan_margin_end_spin.value() == 20


def test_cada_margen_desplaza_su_extremo_en_sentido_contrario(ventana):
    _regenerar(ventana, largo_mm=80, margen_start_mm=0, margen_end_mm=0)
    base_p1 = _texto(ventana, 0, COL_Y)
    base_p2 = _texto(ventana, 1, COL_Y)
    _regenerar(ventana, largo_mm=80, margen_start_mm=30, margen_end_mm=30)
    assert _texto(ventana, 0, COL_Y) == "+0.390"  # 0.420 - 0.030
    assert _texto(ventana, 1, COL_Y) == "+0.530"  # 0.500 + 0.030
    assert base_p1 == "+0.420"
    assert base_p2 == "+0.500"


def test_los_margenes_desplazan_los_extremos_de_forma_independiente(ventana):
    """Mover un margen NO toca el otro extremo."""
    _regenerar(ventana, largo_mm=80, margen_start_mm=0, margen_end_mm=0)
    _regenerar(ventana, largo_mm=80, margen_start_mm=20, margen_end_mm=0)
    assert _texto(ventana, 0, COL_Y) == "+0.400"  # solo P1 se retranquea
    assert _texto(ventana, 1, COL_Y) == "+0.500"  # P2 intacto

    _regenerar(ventana, largo_mm=80, margen_start_mm=0, margen_end_mm=20)
    assert _texto(ventana, 0, COL_Y) == "+0.420"  # P1 intacto
    assert _texto(ventana, 1, COL_Y) == "+0.520"  # solo P2 se avanza


def test_el_margen_no_mueve_la_altura_ni_la_x(ventana):
    _regenerar(ventana, margen_start_mm=25, margen_end_mm=15)
    for fila in (0, 1):
        assert _texto(ventana, fila, COL_X) == "+0.000"
        assert _texto(ventana, fila, COL_Z) == "+0.525"


def test_el_margen_alarga_el_recorrido_de_escaneo(ventana):
    _regenerar(ventana, largo_mm=80, margen_start_mm=0, margen_end_mm=0)
    assert ventana._scan_solution.scan_length == pytest.approx(0.08, abs=1e-12)
    _regenerar(ventana, largo_mm=80, margen_start_mm=30, margen_end_mm=20)
    assert ventana._scan_solution.scan_length == pytest.approx(0.13, abs=1e-12)


def test_los_margenes_sobreviven_a_guardar_y_reabrir(ventana):
    _regenerar(ventana, margen_start_mm=5, margen_end_mm=18)
    ventana.btn_scan_save.click()
    reabierta = gui_mod.MainWindow(ros_node=None)
    assert reabierta._scan_margin_start_spin.value() == 5
    assert reabierta._scan_margin_end_spin.value() == 18
    spec = reabierta._scan_current_spec()
    assert spec.scan_margin_start == pytest.approx(0.005, abs=1e-12)
    assert spec.scan_margin_end == pytest.approx(0.018, abs=1e-12)
