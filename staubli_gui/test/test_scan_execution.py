"""Tests headless de la planificación del barrido (scan_execution).

Sin Qt, sin ROS y sin robot. Comprueban el contrato que hace que el barrido
sea ejecutable de forma fiable:

  1. El barrido es un único segmento de ESCANEO, P1 -> P2, y termina en P2.
  2. No existe punto de retirada (P3) ni segmento RETIRADA.
  3. Los tiempos son estrictamente crecientes y se rechazan si no lo son.
  4. El `q` de cada waypoint es EXACTAMENTE el que dio `resolve_scan()`: no se
     recalcula la IK ni se sustituye la orientación de `scan_path` por
     `look_at_part_center()` (que apunta el +Z de tool0 y no el eje óptico real
     de la cámara).

El punto 4 es el que evita la regresión más grave posible: mandar a ejecutar
una orientación distinta de la calculada para el barrido.
"""

import pytest

from staubli_gui.scan_execution import (
    DEFAULT_T_P1,
    DEFAULT_T_SCAN,
    SegmentoEscaneo,
    build_scan_execution,
    describe_plan,
    waypoints_to_dicts,
)
from staubli_gui.scan_path import (
    ScanPathSpec,
    ScanProfile,
    load_kinematics,
    resolve_scan,
)

CARA = "+Z"
NOMBRE = "test_scan"


@pytest.fixture(scope="module")
def solution():
    """Solución real de barrido (usa la cinemática de verdad, sin ROS)."""
    return resolve_scan(
        ScanProfile(),
        ScanPathSpec(name=NOMBRE, face=CARA, line_axis=[0.0, -1.0, 0.0]),
        load_kinematics(),
    )


@pytest.fixture
def plan(solution):
    return build_scan_execution(solution)


# ── 1. P1 y P2 forman el segmento de escaneo ──────────────────────────

def test_el_plan_tiene_un_unico_segmento_de_escaneo(plan):
    assert isinstance(plan.scan, SegmentoEscaneo)
    assert not hasattr(plan, "retract")


def test_el_escaneo_va_de_P1_a_P2(plan):
    assert plan.scan.start.role == "P1"
    assert plan.scan.end.role == "P2"


def test_el_escaneo_emite_solo_los_dos_extremos_en_la_v1(plan):
    assert [w.role for w in plan.scan.samples()] == ["P1", "P2"]


def test_el_segmento_etiqueta_su_naturaleza(plan):
    assert plan.scan.label == "ESCANEO"


# ── 2. No hay P3 ni RETIRADA: el barrido termina en P2 ──────────────

def test_P3_no_aparece_en_las_muestras_del_escaneo(plan):
    roles_escaneo = [w.role for w in plan.scan.samples()]
    assert "P3" not in roles_escaneo


def test_los_waypoints_totales_son_solo_P1_y_P2(plan):
    assert [w.role for w in plan.waypoints] == ["P1", "P2"]


def test_la_trayectoria_termina_en_P2(plan):
    """Requisito de la versión actual: el barrido acaba en P2."""
    assert plan.waypoints[-1].role == "P2"
    assert plan.total_duration == pytest.approx(plan.scan.end.time_from_start)


def test_ningun_waypoint_esta_marcado_como_retirada(plan):
    assert all(w.segment == "ESCANEO" for w in plan.waypoints)


def test_los_dos_waypoints_estan_marcados_como_escaneo(plan):
    assert [w.segment for w in plan.waypoints] == ["ESCANEO", "ESCANEO"]


def test_el_resumen_solo_describe_el_escaneo(plan):
    lineas = plan.summary_lines()
    assert any("P1 -> P2" in ln and "ESCANEO" in ln for ln in lineas)
    assert not any("RETIRADA" in ln or "P3" in ln for ln in lineas)


def test_el_log_menciona_la_interpolacion_lineal_del_jtc(plan):
    # Es la consecuencia de no emitir velocidades: hay que hacerla explícita.
    assert "lineal" in describe_plan(plan)


# ── 3. Tiempos ───────────────────────────────────────────────────────

def test_los_tiempos_son_estrictamente_crecientes(plan):
    tiempos = [w.time_from_start for w in plan.waypoints]
    assert tiempos == sorted(tiempos)
    assert all(a < b for a, b in zip(tiempos, tiempos[1:]))


def test_los_tiempos_por_defecto_son_0_y_1_0(plan):
    assert [w.time_from_start for w in plan.waypoints] == [
        DEFAULT_T_P1,
        DEFAULT_T_P1 + DEFAULT_T_SCAN,
    ]


def test_la_duracion_del_escaneo_es_correcta(plan):
    assert plan.scan.duration == pytest.approx(DEFAULT_T_SCAN)
    assert plan.total_duration == pytest.approx(1.0)


def test_se_puede_mover_el_tiempo_de_inicio(solution):
    plan = build_scan_execution(solution, t_p1=2.5)
    assert [w.time_from_start for w in plan.waypoints] == [2.5, 3.5]


def test_un_tiempo_de_escaneo_nulo_se_rechaza(solution):
    with pytest.raises(ValueError, match="crecientes"):
        build_scan_execution(solution, t_scan=0.0)


def test_un_tiempo_de_escaneo_negativo_se_rechaza(solution):
    with pytest.raises(ValueError, match="crecientes"):
        build_scan_execution(solution, t_scan=-1.0)


# ── 4. El q es exactamente el de resolve_scan (sin recalcular IK) ───

def test_los_q_son_identicos_bit_a_bit_a_los_de_resolve_scan(plan, solution):
    for waypoint, punto in zip(plan.waypoints, solution.points):
        assert waypoint.q == pytest.approx(punto.q, abs=0.0, rel=0.0)


def test_no_se_deriva_ninguna_articulacion_resuelta(plan, solution):
    for waypoint, punto in zip(plan.waypoints, solution.points):
        assert list(waypoint.joint_positions) == list(punto.q)


def test_las_posiciones_cartesianas_no_se_usan_para_recalcular(plan, solution):
    """La orientación viene dentro del q; el plan no debe reorientar nada."""
    quats = [w.role for w in plan.waypoints]
    assert quats == ["P1", "P2"]
    # Si se reorientase con look_at_part_center, el primer q cambiaría.
    assert plan.scan.start.q[4] == solution.point("P1").q[4]


def test_el_plan_no_depende_de_look_at_part_center():
    """`scan_execution` no debe importar nada de orientación de tool0.

    Se analiza el AST (no el texto) para no dar por buena una mención que solo
    esté en la documentación.
    """
    import ast
    import inspect

    import staubli_gui.scan_execution as mod

    arbol = ast.parse(inspect.getsource(mod))
    importados = set()
    llamadas = set()
    for nodo in ast.walk(arbol):
        if isinstance(nodo, ast.ImportFrom) and nodo.module:
            importados.add(nodo.module)
            importados.update(a.name for a in nodo.names)
        elif isinstance(nodo, ast.Import):
            importados.update(a.name for a in nodo.names)
        elif isinstance(nodo, ast.Attribute):
            llamadas.add(nodo.attr)
        elif isinstance(nodo, ast.Name):
            llamadas.add(nodo.id)

    prohibido = {"look_at_part_center", "orientation_frame_looking_at",
                 "frame_looking_with_optical_axis"}
    assert not (importados & prohibido)
    assert not (llamadas & prohibido)


def test_el_plan_no_llama_a_la_ik():
    """Ni el módulo ni el plan resuelven IK: solo consumen `ScanPoint.q`."""
    import ast
    import inspect

    import staubli_gui.scan_execution as mod

    arbol = ast.parse(inspect.getsource(mod))
    llamadas = {
        nodo.attr if isinstance(nodo, ast.Attribute) else nodo.id
        for nodo in ast.walk(arbol)
        if isinstance(nodo, (ast.Attribute, ast.Name))
    }
    assert "inverse_kinematics" not in llamadas
    assert "forward_kinematics" not in llamadas
    assert "load_kinematics" not in llamadas


def test_los_waypoints_no_llevan_velocidades(plan):
    """v1 sin velocidades -> el JTC interpola linealmente en articular."""
    assert all("velocities" not in wp for wp in waypoints_to_dicts(plan))


def test_los_waypoints_para_el_builder_tienen_la_forma_esperada(plan):
    wps = waypoints_to_dicts(plan)
    assert len(wps) == 2
    for wp in wps:
        assert set(wp) == {"name", "role", "q", "time_from_start", "segment"}
        assert len(wp["q"]) == 6


# ── Validaciones de entrada ───────────────────────────────────────────

def test_una_solucion_invalida_no_se_planifica(solution):
    import copy

    rota = copy.deepcopy(solution)
    rota.points[0].inside_limits = False
    with pytest.raises(ValueError, match="no es válida"):
        build_scan_execution(rota)


def test_una_solucion_sin_IK_no_se_planifica(solution):
    import copy

    rota = copy.deepcopy(solution)
    rota.points[1].solved = False
    with pytest.raises(ValueError, match="no es válida"):
        build_scan_execution(rota)


def test_una_solucion_incompleta_se_rechaza(solution):
    import copy

    corta = copy.deepcopy(solution)
    # El barrido necesita P1 y P2; quedarse solo con P1 no es planificable.
    corta.points = corta.points[:1]
    with pytest.raises(ValueError, match="no es válida"):
        build_scan_execution(corta)


def test_nombres_personalizados_se_respetan(solution):
    plan = build_scan_execution(solution, names={"P1": "ini", "P2": "fin"})
    assert [w.name for w in plan.waypoints] == ["ini", "fin"]


def test_el_waypoint_expone_las_posiciones_articulares_como_floats(plan):
    posiciones = plan.scan.start.joint_positions
    assert all(isinstance(v, float) for v in posiciones)
    assert len(posiciones) == 6
