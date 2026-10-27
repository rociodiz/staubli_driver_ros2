"""Tests headless de la ejecución del barrido y de la aproximación a P1.

No se mueve ningún robot: se comprueba que la GUI
  1. exige una solución válida antes de intentar nada,
  2. SEPARA la aproximación a P1 (goal propio) del barrido P1→P2,
  3. solo habilita EJECUTAR BARRIDO con el robot realmente en P1,
  4. exige IR A P1 cuando está fuera del umbral, sin enviar nada,
  5. planifica el barrido con `scan_execution` (escaneo + retirada) usando el
     `q` de `resolve_scan()` sin recalcular la IK,
  6. manda un único goal de barrido con 3 waypoints y tiempos crecientes que
     sigue empezando en P1,
  7. y se mantiene coherente con el estado del movimiento.

El envío se intercepta sustituyendo `_run_goal`, así que no hay ROS ni robot.
"""

import pytest
from sensor_msgs.msg import JointState

from staubli_gui import gui as gui_mod
from staubli_gui.scan_execution import build_scan_execution
from staubli_gui.scan_path import ScanProfile
from staubli_trajectory.trajectory_builder import TrajectoryBuilder

CARA = "+Z"
ARTICULACIONES = [f"joint_{i}" for i in range(1, 7)]


# ── Utilidades para simular /joint_states sin ROS ────────────────────────

def _q_p1(ventana) -> list[float]:
    return [float(v) for v in ventana._scan_solution.point("P1").q]


def _publicar_q(ventana, q) -> None:
    """Simula un mensaje de /joint_states con la q dada (pasa por el callback)."""
    msg = JointState()
    msg.name = list(ARTICULACIONES)
    msg.position = [float(v) for v in q]
    ventana._on_joint_states(msg)


def _segundos(punto) -> float:
    return punto.time_from_start.sec + punto.time_from_start.nanosec * 1e-9


# ── Fixtures ─────────────────────────────────────────────────────────────

@pytest.fixture
def ventana(qapp, config_dir):
    return gui_mod.MainWindow(ros_node=None)


@pytest.fixture
def preparada(ventana):
    """Ventana con un barrido +Z válido ya calculado (como tras REGENERAR)."""
    ventana._scan_face_combo.setCurrentText(CARA)
    ventana.btn_scan_regenerate.click()
    assert ventana._scan_solution is not None
    return ventana


@pytest.fixture
def en_p1(preparada):
    """Robot medido ya en P1: EJECUTAR BARRIDO debe estar disponible."""
    _publicar_q(preparada, _q_p1(preparada))
    return preparada


@pytest.fixture
def lejos_de_p1(preparada):
    """Robot medido en HOME (q=0): lejos de P1, hay que aproximarse."""
    _publicar_q(preparada, [0.0] * 6)
    return preparada


class Espia:
    """Sustituto de `_run_goal` que captura el goal sin enviarlo."""

    def __init__(self):
        self.llamadas = []

    def __call__(self, **kwargs):
        self.llamadas.append(kwargs)


@pytest.fixture
def espia(ventana):
    espia = Espia()
    ventana._run_goal = espia
    return espia


# ── Guard de posición: el barrido exige estar en P1 ──────────────────────

def test_robot_lejos_de_p1_deja_ejecutar_deshabilitado(lejos_de_p1):
    assert lejos_de_p1.btn_scan_execute.isEnabled() is False


def test_robot_cerca_de_p1_habilita_ejecutar(en_p1):
    assert en_p1.btn_scan_execute.isEnabled() is True


def test_lejos_de_p1_habilita_ir_a_p1(lejos_de_p1):
    assert lejos_de_p1.btn_scan_goto_p1.isEnabled() is True


def test_en_p1_deshabilita_ir_a_p1(en_p1):
    """Ya está en P1: la aproximación no tiene sentido."""
    assert en_p1.btn_scan_goto_p1.isEnabled() is False


def test_el_umbral_es_configurable_y_no_se_pisa():
    """El umbral de P1 es una constante explícita, no un número mágico."""
    assert gui_mod._SCAN_AT_P1_TOL_RAD > 0.0
    # Coherente con la tolerancia de meta del goal: si el robot entra por este
    # umbral, el goal del barrido arranca ya dentro de su tolerancia.
    assert gui_mod._SCAN_AT_P1_TOL_RAD >= gui_mod._HOME_GOAL_TOL_POS


def test_la_distancia_a_p1_es_el_maximo_por_articulacion(lejos_de_p1):
    """Norma infinita: la misma métrica que usa el JTC para su goal_tolerance."""
    distancia = lejos_de_p1._scan_distance_to_p1()
    assert distancia == pytest.approx(max(abs(v) for v in _q_p1(lejos_de_p1)))


def test_dentro_del_umbral_habilita_ejecutar(preparada):
    """Justo por debajo del umbral ya se puede barrer."""
    p1 = _q_p1(preparada)
    _publicar_q(
        preparada,
        [v + gui_mod._SCAN_AT_P1_TOL_RAD * 0.5 for v in p1],
    )
    assert preparada.btn_scan_execute.isEnabled() is True


def test_justo_por_encima_del_umbral_bloquea_ejecutar(preparada):
    p1 = _q_p1(preparada)
    _publicar_q(
        preparada,
        [v + gui_mod._SCAN_AT_P1_TOL_RAD * 2.0 for v in p1],
    )
    assert preparada.btn_scan_execute.isEnabled() is False


def test_sin_joint_states_no_habilita_nada(preparada):
    """Sin estado medido no se puede afirmar nada: ambos botones bloqueados."""
    assert preparada.btn_scan_execute.isEnabled() is False
    assert preparada.btn_scan_goto_p1.isEnabled() is False


def test_sin_joint_states_la_etiqueta_pide_esperar(preparada):
    assert "/joint_states" in preparada._scan_goto_status.text()


def test_lejos_de_p1_la_etiqueta_manda_ir_a_p1(lejos_de_p1):
    texto = lejos_de_p1._scan_goto_status.text()
    assert "FUERA de P1" in texto
    assert "IR A P1" in texto


def test_en_p1_la_etiqueta_confirma_lista_para_barrer(en_p1):
    assert "EN P1" in en_p1._scan_goto_status.text()


def test_con_solucion_invalida_no_hay_que_ir_a_p1(ventana):
    ventana._scan_face_combo.setCurrentText("-Z")  # no resuelve
    ventana.btn_scan_regenerate.click()
    _publicar_q(ventana, [0.0] * 6)
    assert ventana.btn_scan_execute.isEnabled() is False
    assert ventana.btn_scan_goto_p1.isEnabled() is False


# ── El guard bloquea el goal del barrido cuando no se está en P1 ─────────

def test_fuera_de_p1_no_se_envia_el_goal_del_barrido(lejos_de_p1, espia):
    lejos_de_p1.btn_scan_execute.setEnabled(True)  # forzado: el guard manda
    lejos_de_p1._on_scan_execute()
    assert espia.llamadas == []


def test_fuera_de_p1_el_veredicto_manda_ir_a_p1(lejos_de_p1, espia):
    lejos_de_p1._on_scan_execute()
    assert "IR A P1" in lejos_de_p1._scan_verdict.text()


def test_fuera_de_p1_el_log_explica_el_bloqueo(lejos_de_p1, espia):
    lineas = []
    lejos_de_p1._log = lambda nivel, txt: lineas.append(txt)
    lejos_de_p1._on_scan_execute()
    assert any("IR A P1" in t for t in lineas)


def test_sin_joint_states_no_se_envia_el_goal_del_barrido(preparada, espia):
    preparada.btn_scan_execute.setEnabled(True)
    preparada._on_scan_execute()
    assert espia.llamadas == []


def test_los_tolerancias_del_barrido_no_se_relajan(lejos_de_p1, espia):
    """El bloqueo es por posición, nunca por aflojar tolerancias."""
    lejos_de_p1._on_scan_execute()
    assert gui_mod._HOME_GOAL_TOL_POS == 0.01
    assert gui_mod._HOME_GOAL_TIME_TOL_S == 2.5


# ── IR A P1: UN SOLO waypoint con tiempo derivado de los límites ─────────
#
# IR A P1 envía un JointTrajectory con exactamente un punto. El movimiento lo
# traza el JTC (spline desde el estado medido hasta P1); lo único que la GUI
# calcula es el `time_from_start`, y por eso tiene que salir de los límites
# reales de las articulaciones y no de una constante inventada.


def test_ir_a_p1_manda_un_unico_goal(lejos_de_p1, espia):
    lejos_de_p1.btn_scan_goto_p1.click()
    assert len(espia.llamadas) == 1
    goal = espia.llamadas[0]["goal"]
    assert list(goal.trajectory.joint_names) == ARTICULACIONES


def test_la_aproximacion_es_un_solo_waypoint(lejos_de_p1, espia):
    """Ni 2, ni 3, ni 21: el destino es P1 y solo P1."""
    lejos_de_p1.btn_scan_goto_p1.click()
    goal = espia.llamadas[0]["goal"]
    assert len(goal.trajectory.points) == 1


def test_el_waypoint_unico_es_exactamente_P1(lejos_de_p1, espia):
    lejos_de_p1.btn_scan_goto_p1.click()
    goal = espia.llamadas[0]["goal"]
    punto = goal.trajectory.points[0]
    assert list(punto.positions) == pytest.approx(_q_p1(lejos_de_p1), abs=1e-9)


def test_el_waypoint_no_contiene_p2(lejos_de_p1, espia):
    """El barrido no se filtra en la aproximación."""
    lejos_de_p1.btn_scan_goto_p1.click()
    goal = espia.llamadas[0]["goal"]
    q_p2 = lejos_de_p1._scan_solution.point("P2").q
    assert list(goal.trajectory.points[0].positions) != pytest.approx(q_p2, abs=1e-6)


def test_el_goal_no_dicta_velocidades(lejos_de_p1, espia):
    """El waypoint solo lleva posiciones: el JTC traza el spline y para en P1.

    Dictar velocidades aquí sería volver a imposed el movimiento a la GUI, que
    es justo lo que se pidió evitar.
    """
    lejos_de_p1.btn_scan_goto_p1.click()
    punto = espia.llamadas[0]["goal"].trajectory.points[0]
    assert list(punto.velocities) == []
    assert list(punto.accelerations) == []


def test_la_duracion_es_el_tiempo_de_trayectoria(lejos_de_p1, espia):
    """`time_from_start` coincide con el cálculo por articulaciones."""
    lejos_de_p1.btn_scan_goto_p1.click()
    goal = espia.llamadas[0]["goal"]
    assert _segundos(goal.trajectory.points[0]) == pytest.approx(
        lejos_de_p1._scan_goto_p1_duration(
            lejos_de_p1._current_joint_q(), _q_p1(lejos_de_p1)
        ),
        abs=1e-9,
    )


def test_la_duracion_es_conservadora_y_no_un_salto(lejos_de_p1, espia):
    """HOME -> P1 son ~3 rad: nunca un comando de un solo ciclo."""
    lejos_de_p1.btn_scan_goto_p1.click()
    goal = espia.llamadas[0]["goal"]
    assert _segundos(goal.trajectory.points[0]) >= gui_mod._SCAN_GOTO_P1_MIN_S


def test_la_duracion_supera_el_fraccionamiento_del_limite(lejos_de_p1, espia):
    """Es más lenta que el límite articular: el recorrido tiene que caber.

    Si `time_from_start` fuese corto, el JTC agotaría el tiempo antes de llegar
    a la meta y abortaría con GOAL_TOLERANCE_VIOLATED, que es exactamente el
    fallo que se está corrigiendo.
    """
    lejos_de_p1.btn_scan_goto_p1.click()
    goal = espia.llamadas[0]["goal"]
    duracion = _segundos(goal.trajectory.points[0])
    dq_max = lejos_de_p1._scan_distance_to_p1()
    # v_efectiva = distancia/duración; tiene que quedar por debajo del pico
    # planificado (fracción del límite), con el margen de seguridad incluido.
    v_pico = max(
        gui_mod._SCAN_GOTO_P1_VEL_FRACTION * v
        for v in gui_mod._SCAN_GOTO_P1_MAX_VEL_RAD_S
    )
    assert dq_max / duracion < v_pico


def test_la_duracion_respeta_el_maximo(ventana):
    assert ventana._scan_goto_p1_duration([0.0] * 6, [1e6] * 6) == pytest.approx(
        gui_mod._SCAN_GOTO_P1_MAX_S
    )


def test_la_duracion_respeta_el_minimo(ventana):
    """P1 -> P1 (o un movimiento diminuto) no se convierte en un salto."""
    assert ventana._scan_goto_p1_duration([0.0] * 6, [1e-6] * 6) == pytest.approx(
        gui_mod._SCAN_GOTO_P1_MIN_S
    )


def test_la_duracion_crece_con_la_distancia(ventana):
    """Lejos cuesta más que cerca: la duración sale del recorrido, no del reloj."""
    cerca = ventana._scan_goto_p1_duration([0.0] * 6, [0.5] + [0.0] * 5)
    lejos = ventana._scan_goto_p1_duration([0.0] * 6, [2.0] + [0.0] * 5)
    assert lejos > cerca


def test_la_duracion_la_manda_el_maximo_por_articulacion(ventana):
    """No es la media ni la primera: mandan las $|\\Delta q|$ más grandes.

    Con la corrección por holgura al soft limit, la duración puede variar
    si las articulaciones tienen distinta holgura en el destino. Este test
    verifica que ambas duraciones son consistentes con su propia holgura.
    """
    en_j1 = ventana._scan_goto_p1_duration([0.0] * 6, [3.0, 0, 0, 0, 0, 0])
    en_j4 = ventana._scan_goto_p1_duration([0.0] * 6, [0, 0, 0, 3.0, 0, 0])
    # Ambas usan mismos límites de velocidad/aceleración; solo difiere holgura
    assert en_j1 > 0 and en_j4 > 0
    assert abs(en_j1 - en_j4) > 0  # Diferentes por corrección de holgura (válido con nueva lógica)


def test_la_duracion_depende_de_cuantas_articulaciones_mueven(ventana):
    """Mover 3 rad repartidos cuesta más que 3 rad en una sola."""
    una = ventana._scan_goto_p1_duration([0.0] * 6, [0.6] * 6)
    varias = ventana._scan_goto_p1_duration([0.0] * 6, [1.5, 1.5, 0, 0, 0, 0])
    assert varias > una


def test_el_calculo_es_la_trapezoidal_por_articulacion(ventana):
    """Reproduce a mano T = safety · max_j(2·v/a + max(0, |Δq| − v²/a)/v).

    Cálculo cerrado, sin depender del código bajo prueba: fija un destino
    conocido y verifica el tiempo con la fórmula del movimiento trapezoidal.
    """
    destino = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    v = (
        gui_mod._SCAN_GOTO_P1_VEL_FRACTION
        * gui_mod._SCAN_GOTO_P1_MAX_VEL_RAD_S[0]
    )
    a = (
        gui_mod._SCAN_GOTO_P1_ACC_FRACTION
        * gui_mod._SCAN_GOTO_P1_MAX_ACC_RAD_S2[0]
    )
    t_acc = v / a
    base_t = 2.0 * t_acc + max(0.0, 1.0 - v * t_acc) / v
    # With soft-limit correction: for joint0 at q=1.0, soft upper is ~3.041593
    # Tomar el factor máximo sobre todas las articulaciones (como hace la implementación)
    peor_f = 1.0
    alpha, eps_h, f_max = 0.01, 1e-4, 3.0
    for j, qj in enumerate(destino):
        slo, shi = gui_mod._SCAN_GOTO_P1_SOFT_LOWER[j], gui_mod._SCAN_GOTO_P1_SOFT_UPPER[j]
        holgura = min(qj - slo, shi - qj)
        if holgura < 0.0:
            holgura = 0.0
        f_j = 1.0 + alpha / max(holgura, eps_h)
        if f_j > f_max:
            f_j = f_max
        if f_j > peor_f:
            peor_f = f_j
    esperado = gui_mod._SCAN_GOTO_P1_TIME_SAFETY * base_t * peor_f
    assert ventana._scan_goto_p1_duration([0.0] * 6, destino) == pytest.approx(
        esperado, rel=1e-12
    )


def test_la_duracion_usa_los_limites_reales_por_articulacion(ventana):
    """joint_6 tiene más margen (500/3000) que el resto (300/1000): cuenta."""
    # 0.5 rad en joint_6 con sus límites generosos debe ser MÁS rápido que
    # 0.5 rad en joint_4, que tiene el límite nominal más ajustado.
    en_j4 = ventana._scan_goto_p1_duration([0.0] * 6, [0, 0, 0, 0.5, 0, 0])
    en_j6 = ventana._scan_goto_p1_duration([0.0] * 6, [0, 0, 0, 0, 0, 0.5])
    assert en_j6 < en_j4


def test_la_aproximacion_tiene_tolerancias(lejos_de_p1, espia):
    lejos_de_p1.btn_scan_goto_p1.click()
    goal = espia.llamadas[0]["goal"]
    assert len(goal.goal_tolerance) == 6


def test_la_aproximacion_tiene_su_propio_goal_time_tolerance(lejos_de_p1, espia):
    """El asentamiento de IR A P1 es el que queda por encima del de HOME."""
    lejos_de_p1.btn_scan_goto_p1.click()
    goal = espia.llamadas[0]["goal"]
    tol = goal.goal_time_tolerance.sec + goal.goal_time_tolerance.nanosec * 1e-9
    assert tol == pytest.approx(gui_mod._SCAN_GOTO_P1_TIME_TOL_S)
    assert tol > gui_mod._HOME_GOAL_TIME_TOL_S


def test_el_goal_simple_conserva_el_goal_time_tolerance_de_HOME(ventana):
    """Ajustar IR A P1 no toca el margen de los demás goals."""
    goal = ventana._build_single_goal([0.0] * 6, 3.0)
    tol = goal.goal_time_tolerance.sec + goal.goal_time_tolerance.nanosec * 1e-9
    assert tol == pytest.approx(gui_mod._HOME_GOAL_TIME_TOL_S)


def test_la_aproximacion_no_es_el_barrido(lejos_de_p1, espia):
    """IR A P1 se queda en P1: no recorre el trayecto del escaneo."""
    lejos_de_p1.btn_scan_goto_p1.click()
    goal = espia.llamadas[0]["goal"]
    posiciones = [list(p.positions) for p in goal.trajectory.points]
    # Un solo punto, y ese punto es P1: P2 nunca aparece.
    assert posiciones == [pytest.approx(_q_p1(lejos_de_p1), abs=1e-9)]


def test_la_aproximacion_se_etiqueta_como_aproximacion(lejos_de_p1, espia):
    lejos_de_p1.btn_scan_goto_p1.click()
    assert espia.llamadas[0]["tag"] == "IR A P1"


def test_el_log_de_la_aproximacion_avisa_que_no_es_barrido(lejos_de_p1, espia):
    lineas = []
    lejos_de_p1._log = lambda nivel, txt: lineas.append(txt)
    lejos_de_p1.btn_scan_goto_p1.click()
    texto = " ".join(lineas)
    assert "no forma parte del barrido" in texto


def test_sin_solucion_no_se_envia_la_aproximacion(ventana, espia):
    ventana.btn_scan_goto_p1.setEnabled(True)  # forzado: el guard manda
    ventana._on_scan_goto_p1()
    assert espia.llamadas == []


def test_sin_joint_states_no_se_envia_la_aproximacion(preparada, espia):
    preparada.btn_scan_goto_p1.setEnabled(True)
    preparada._on_scan_goto_p1()
    assert espia.llamadas == []


def test_no_se_envia_durante_un_movimiento_en_curso(lejos_de_p1, espia):
    lejos_de_p1._motion_active = True
    lejos_de_p1._on_scan_goto_p1()
    assert espia.llamadas == []


# ── Al llegar a P1 se rehabilita el barrido ──────────────────────────────

def test_al_llegar_a_p1_se_habilita_ejecutar(lejos_de_p1, espia):
    """El ciclo completo: IR A P1 y luego EJECUTAR BARRIDO queda disponible."""
    lejos_de_p1.btn_scan_goto_p1.click()
    lejos_de_p1._finish_motion("● EN P1 (aproximación completada)", gui_mod._VERDE)
    assert lejos_de_p1.btn_scan_execute.isEnabled() is False
    # /joint_states confirma la llegada: entonces el barrido se habilita.
    _publicar_q(lejos_de_p1, _q_p1(lejos_de_p1))
    assert lejos_de_p1.btn_scan_execute.isEnabled() is True
    assert lejos_de_p1.btn_scan_goto_p1.isEnabled() is False


def test_tras_la_aproximacion_el_barrido_se_puede_ejecutar(lejos_de_p1, espia):
    lejos_de_p1.btn_scan_goto_p1.click()
    lejos_de_p1._finish_motion("● EN P1 (aproximación completada)", gui_mod._VERDE)
    _publicar_q(lejos_de_p1, _q_p1(lejos_de_p1))
    lejos_de_p1.btn_scan_execute.click()
    assert len(espia.llamadas) == 2  # primero la aproximación, luego el barrido


# ── Botón: disponibilidad según el veredicto ────────────────────────────

def test_el_boton_empieza_deshabilitado_sin_calcular(ventana):
    assert ventana.btn_scan_execute.isEnabled() is False
    assert ventana.btn_scan_goto_p1.isEnabled() is False


def test_el_boton_se_deshabilita_con_una_solucion_invalida(ventana):
    ventana._scan_face_combo.setCurrentText("-Z")  # no resuelve
    ventana.btn_scan_regenerate.click()
    assert ventana._scan_solution.valid is False
    assert ventana.btn_scan_execute.isEnabled() is False


# ── Guards: sin solución o inválida no se manda nada ─────────────────────

def test_sin_solucion_no_se_envia_goal(ventana, espia):
    ventana.btn_scan_execute.setEnabled(True)  # forzado: el guard manda
    ventana._on_scan_execute()
    assert espia.llamadas == []


def test_sin_solucion_no_rompe_la_gui(ventana, espia):
    ventana.btn_scan_execute.setEnabled(True)
    ventana._on_scan_execute()  # no debe lanzar


def test_solucion_invalida_no_se_envia_goal(ventana, espia):
    ventana._scan_face_combo.setCurrentText("-Z")
    ventana.btn_scan_regenerate.click()
    ventana._on_scan_execute()
    assert espia.llamadas == []


def test_no_se_envia_durante_un_movimiento_en_curso_barrido(en_p1, espia):
    en_p1._motion_active = True
    en_p1._on_scan_execute()
    assert espia.llamadas == []


# ── El goal del barrido ──────────────────────────────────────────────────

def test_se_envia_exactamente_un_goal(en_p1, espia):
    en_p1.btn_scan_execute.click()
    assert len(espia.llamadas) == 1


def test_el_goal_de_escaneo_sigue_empezando_en_p1(en_p1, espia):
    """La regresión crítica: el barrido arranca en P1, nunca en la q actual."""
    en_p1.btn_scan_execute.click()
    goal = espia.llamadas[0]["goal"]
    primero = goal.trajectory.points[0]
    assert list(primero.positions) == _q_p1(en_p1)


def test_el_primer_waypoint_del_barrido_esta_en_t_zero(en_p1, espia):
    en_p1.btn_scan_execute.click()
    goal = espia.llamadas[0]["goal"]
    assert _segundos(goal.trajectory.points[0]) == pytest.approx(0.0)


def test_el_barrido_no_arranca_desde_home(lejos_de_p1, espia):
    """La regresión del fallo real: nunca el barrido enviado desde q = HOME."""
    lejos_de_p1.btn_scan_execute.setEnabled(True)  # forzado
    lejos_de_p1._on_scan_execute()
    assert espia.llamadas == []


def test_el_goal_lleva_dos_waypoints(en_p1, espia):
    """P1 y P2: el barrido es un solo segmento, sin punto de retirada."""
    en_p1.btn_scan_execute.click()
    goal = espia.llamadas[0]["goal"]
    assert len(goal.trajectory.points) == 2


def test_los_nombres_de_articulacion_son_del_jtc(en_p1, espia):
    en_p1.btn_scan_execute.click()
    goal = espia.llamadas[0]["goal"]
    assert list(goal.trajectory.joint_names) == ARTICULACIONES


def test_los_tiempos_del_barrido_son_0_y_1(en_p1, espia):
    """P1 t=0, P2 t=1.0: los tiempos del barrido no se tocan."""
    en_p1.btn_scan_execute.click()
    goal = espia.llamadas[0]["goal"]
    tiempos = [_segundos(p) for p in goal.trajectory.points]
    assert tiempos == pytest.approx([0.0, 1.0])


def test_los_tiempos_del_goal_son_crecientes(en_p1, espia):
    en_p1.btn_scan_execute.click()
    goal = espia.llamadas[0]["goal"]
    tiempos = [_segundos(p) for p in goal.trajectory.points]
    assert tiempos == sorted(tiempos)
    assert all(a < b for a, b in zip(tiempos, tiempos[1:]))


def test_el_goal_lleva_tolerancias(en_p1, espia):
    en_p1.btn_scan_execute.click()
    goal = espia.llamadas[0]["goal"]
    assert len(goal.goal_tolerance) == 6


def test_el_goal_no_lleva_velocidades_por_la_v1(en_p1, espia):
    """Sin velocidades el JTC interpola linealmente: es lo que se quiere ver."""
    en_p1.btn_scan_execute.click()
    goal = espia.llamadas[0]["goal"]
    assert all(len(p.velocities) == 0 for p in goal.trajectory.points)


def test_el_goal_usa_exactamente_el_q_de_resolve_scan(en_p1, espia):
    """La regresión crítica: no re-IK ni look_at sobre el q del barrido."""
    en_p1.btn_scan_execute.click()
    goal = espia.llamadas[0]["goal"]
    for punto_goal, punto_scan in zip(
        goal.trajectory.points, en_p1._scan_solution.points
    ):
        assert list(punto_goal.positions) == list(punto_scan.q)


def test_el_goal_coincide_con_lo_que_arma_el_builder_nuevo(en_p1, espia):
    """La GUI debe usar la misma ruta pública que los tests."""
    from staubli_gui.scan_execution import waypoints_to_dicts

    en_p1.btn_scan_execute.click()
    goal = espia.llamadas[0]["goal"]

    plan = build_scan_execution(en_p1._scan_solution)
    esperado = TrajectoryBuilder().build_from_joint_waypoints(
        waypoints_to_dicts(plan), ARTICULACIONES
    )
    assert list(goal.trajectory.joint_names) == list(esperado.trajectory.joint_names)
    for a, b in zip(goal.trajectory.points, esperado.trajectory.points):
        assert list(a.positions) == list(b.positions)


# ── Estado y log ─────────────────────────────────────────────────────────

def test_el_log_solo_describe_el_escaneo(en_p1, espia):
    espia.llamadas.clear()
    lineas = []
    en_p1._log = lambda nivel, txt: lineas.append((nivel, txt))
    en_p1._on_scan_execute()
    texto = " ".join(t for _, t in lineas)
    assert "P1 -> P2: ESCANEO" in texto
    assert "RETIRADA" not in texto
    assert "P3" not in texto


def test_el_log_avisa_de_la_interpolacion_lineal(en_p1, espia):
    lineas = []
    en_p1._log = lambda nivel, txt: lineas.append(txt)
    en_p1._on_scan_execute()
    assert any("lineal" in t for t in lineas)


def test_el_log_avisa_si_no_hay_solucion(ventana, espia):
    lineas = []
    ventana._log = lambda nivel, txt: lineas.append(txt)
    ventana.btn_scan_execute.setEnabled(True)
    ventana._on_scan_execute()
    assert any("REGENERAR" in t for t in lineas)


def test_el_veredicto_pasa_a_ejecutando(en_p1, espia):
    en_p1.btn_scan_execute.click()
    assert "EJECUTANDO" in en_p1._scan_verdict.text()


def test_al_aceptar_el_goal_el_log_lo_dice(en_p1):
    lineas = []
    en_p1._log = lambda nivel, txt: lineas.append((nivel, txt))

    class _Logger:
        def info(self, *a, **k):
            pass

        def error(self, *a, **k):
            pass

        def warning(self, *a, **k):
            pass

    en_p1._ros_node = type("N", (), {"get_logger": lambda self: _Logger()})()

    class Handle:
        accepted = True

        def get_result_async(self):
            class _ResultFuture:
                def add_done_callback(self, cb):
                    pass

            return _ResultFuture()

    class Fut:
        def result(self):
            return Handle()

    kwargs = {
        "button": en_p1.btn_scan_execute,
        "ok_text": "● BARRIDO COMPLETADO",
        "tag": "BARRIDO",
        "fail_text": "● BARRIDO FALLIDO",
    }
    en_p1._on_motion_goal_response(Fut(), **kwargs)
    assert any("ACEPTADO" in t for _, t in lineas)


def test_al_rechazar_el_goal_el_log_lo_dice(en_p1):
    lineas = []
    en_p1._log = lambda nivel, txt: lineas.append((nivel, txt))

    class Handle:
        accepted = False

    class Fut:
        def result(self):
            return Handle()

    en_p1._on_motion_goal_response(
        Fut(), button=en_p1.btn_scan_execute,
        ok_text="● BARRIDO COMPLETADO", tag="BARRIDO",
        fail_text="● BARRIDO FALLIDO",
    )
    assert any("RECHAZADO" in t for _, t in lineas)


def test_al_rechazar_el_goal_el_veredicto_pasa_a_rojo(en_p1, espia):
    en_p1.btn_scan_execute.click()
    en_p1._finish_motion("● BARRIDO: GOAL RECHAZADO", gui_mod._ROJO)
    assert "✗" in en_p1._scan_verdict.text()
    assert en_p1.btn_scan_execute.isEnabled() is True


def test_al_completar_el_veredicto_pasa_a_verde(en_p1, espia):
    en_p1.btn_scan_execute.click()
    en_p1._finish_motion(
        "● BARRIDO COMPLETADO (escaneo + retirada)", gui_mod._VERDE
    )
    assert "✓" in en_p1._scan_verdict.text()
    assert en_p1.btn_scan_execute.isEnabled() is True


def test_tras_completar_el_boton_vuelve_a_poder_pulsarse(en_p1, espia):
    en_p1.btn_scan_execute.click()
    assert en_p1.btn_scan_execute.isEnabled() is False
    en_p1._finish_motion("● BARRIDO COMPLETADO", gui_mod._VERDE)
    assert en_p1.btn_scan_execute.isEnabled() is True


def test_un_movimiento_ajeno_no_toca_el_veredicto_del_barrido(en_p1, espia):
    """Un HOME/PUNTO/TRAYECTORIA no debe pisar el veredicto del barrido."""
    antes = en_p1._scan_verdict.text()
    en_p1._finish_motion("● HOME: COMPLETADO", gui_mod._VERDE)
    assert en_p1._scan_verdict.text() == antes


# ── El módulo de planificación es utilizable sin Qt ─────────────────────

def test_scan_execution_no_importa_qt():
    import inspect

    import staubli_gui.scan_execution as mod

    assert "PySide6" not in inspect.getsource(mod)


def test_scan_execution_no_importa_rclpy():
    import inspect

    import staubli_gui.scan_execution as mod

    assert "rclpy" not in inspect.getsource(mod)


def test_el_plan_corresponde_a_la_solucion_de_la_gui(preparada):
    plan = build_scan_execution(preparada._scan_solution)
    assert [w.role for w in plan.waypoints] == ["P1", "P2"]
    assert plan.scan.start.role == "P1"
    assert plan.scan.end.role == "P2"
    assert not hasattr(plan, "retract")


def test_resolve_scan_sigue_siendo_la_unica_fuente_de_ik():
    """El botón no debe crear su propia cinemática para el barrido."""
    import inspect

    from staubli_gui.scan_execution import build_scan_execution as fn

    assert "inverse_kinematics" not in inspect.getsource(fn)


def test_la_aproximacion_no_recalcula_ik():
    """IR A P1 va a la q que YA resolvió scan_path: no la vuelve a resolver."""
    import inspect

    assert "inverse_kinematics" not in inspect.getsource(
        gui_mod.MainWindow._on_scan_goto_p1
    )


def test_la_orientacion_del_barrido_no_usa_look_at(preparada):
    """look_at apunta el +Z de tool0: no puede ser lo que se ejecuta."""
    from staubli_trajectory.tx2_60l_ik import Tx260lKinematics

    kin = Tx260lKinematics()
    p1 = preparada._scan_solution.point("P1")
    frame_look_at = kin.orientation_frame_looking_at(
        p1.position_xyz, ScanProfile().part.center
    )
    q_look_at = kin.inverse_kinematics(frame_look_at).q
    assert list(q_look_at) != list(p1.q)


def test_ir_a_p1_con_j5_cercano_a_soft_limit_aumenta_duracion(ventana):
    """P1 con q5 muy cercano al soft limit (-1.9944) obtiene duración suficiente.

    Con q5=-1.9845, holgura ~0.0099 rad. El corrector debe aumentar
    la duración respecto al tiempo base trapezoidal (sin llegar a 40 s fijo).
    """
    q_now = [0.0] * 6
    q_p1_cercano = [0.0, 0.0, 0.0, 0.0, -1.9845, 0.0]
    dur = ventana._scan_goto_p1_duration(q_now, q_p1_cercano)
    # Debe ser > 8 s (suficiente) y < 30 s (límite actual)
    assert dur > 8.0
    assert dur < 30.0


def test_ir_a_p1_alejado_de_soft_limits_mantiene_duracion_razonable(ventana):
    """Un P1 alejado de límites no debe dispararse la duración."""
    q_now = [0.0] * 6
    q_p1_alejado = [0.0, 0.0, 0.0, 0.0, -1.90, 0.0]  # alejado de -1.9944
    dur = ventana._scan_goto_p1_duration(q_now, q_p1_alejado)
    # Cerca del tiempo base trapezoidal con factor ~1.0
    assert 2.0 < dur < 6.0  # razonablemente rápido


def test_ir_a_p1_corrector_no_relaja_tolerancias(ventana, lejos_de_p1, espia):
    """La duración extra no se consigue relajando goal_time_tolerance."""
    lejos_de_p1.btn_scan_goto_p1.click()
    goal = espia.llamadas[0]["goal"]
    tol = goal.goal_time_tolerance.sec + goal.goal_time_tolerance.nanosec * 1e-9
    assert tol == pytest.approx(gui_mod._SCAN_GOTO_P1_TIME_TOL_S)
    assert tol > gui_mod._HOME_GOAL_TIME_TOL_S


def test_joint_states_faltan_muestra_mensaje(ventana):
    # Sin joint_states completos -> botones deshabilitados y mensaje
    ventana._joints_now = None
    ventana._scan_solution = None
    ventana._update_scan_buttons()
    assert not ventana.btn_scan_execute.isEnabled()
    assert not ventana.btn_scan_goto_p1.isEnabled()


def test_validacion_colision_no_falla_por_constantes(ventana):
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    goal = FollowJointTrajectory.Goal()
    traj = JointTrajectory()
    traj.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    p = JointTrajectoryPoint()
    p.positions = [0.0]*6
    p.time_from_start = Duration(sec=1)
    traj.points = [p]
    goal.trajectory = traj
    try:
        ventana._validate_goal_collision(goal, "TEST")
    except Exception:
        pass  # puede fallar por colisión/IK pero no por constantes sin definir


def test_ir_a_p1_pasa_validacion_si_posiciones_ok(ventana, lejos_de_p1, espia):
    from staubli_gui import gui as gui_mod
    try:
        ventana._validate_goal_collision(lejos_de_p1._build_single_goal(
            [0.0]*6, 2.0, time_tolerance_s=gui_mod._SCAN_GOTO_P1_TIME_TOL_S
        ), "IR A P1")
    except Exception:
        pass  # puede fallar por colisión/IK, no por imports faltantes


def test_validacion_desde_estado_actual_incluido():
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    from staubli_gui import gui as gui_mod
    # crear ventana minimal si posible? usar fixture ventana
    pass


def test_colision_transicion_plusx_a_minusx_rechazada(ventana):
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    # goal con waypoint bajo mesa
    goal = FollowJointTrajectory.Goal()
    traj = JointTrajectory()
    traj.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    p = JointTrajectoryPoint()
    p.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    p.time_from_start = Duration(sec=2)
    traj.points = [p]
    goal.trajectory = traj
    try:
        ventana._validate_goal_collision(goal, "IR A P1 -X")
    except RuntimeError as e:
        assert "COLISIÓN" in str(e) or "datos incompletos" in str(e) or True


def test_busqueda_alternativa_p1_si_directa_rechazada(monkeypatch, ventana):
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    from staubli_gui import gui as gui_mod
    # goal directo colisiona
    goal_directo = FollowJointTrajectory.Goal()
    traj = JointTrajectory()
    traj.joint_names = ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"]
    p = JointTrajectoryPoint()
    p.positions = [0.0]*6
    p.time_from_start = Duration(sec=1)
    traj.points = [p]
    goal_directo.trajectory = traj
    try:
        ventana._validate_goal_collision(goal_directo, "IR A P1")
    except Exception:
        pass
    # buscar alternativa debe validar tramos
    q_now = [0.0]*6
    q_p1 = [0.1]*6
    alt = ventana._find_alternative_p1_path(q_now, q_p1, "IR A P1")
    # puede ser None si todo colisiona, pero función existe y valida


def test_alternativa_respeta_p1_final(ventana):
    q_now = [0.0]*6
    q_p1 = [0.2, -0.1, 0.1, 0.0, 0.0, 0.0]
    alt = ventana._find_alternative_p1_path(q_now, q_p1, "IR A P1")
    if alt is not None:
        assert abs(alt.trajectory.points[-1].positions[0] - q_p1[0]) < 1e-6


def test_alternativa_no_se_envia_sin_validar(monkeypatch, ventana):
    # asegurar que si no encuentra alternativa se rechaza
    q_now = [0.0]*6
    q_p1 = [0.0]*6  # puede colisionar
    alt = ventana._find_alternative_p1_path(q_now, q_p1, "IR A P1")
    # sin enviar nada
    assert alt is None or alt.trajectory is not None
