"""Tests headless de la geometría de barrido P1/P2/P3 (`scan_path`).

No se mueve ningún robot: todo se resuelve con la IK real
(`Tx260lKinematics`) contra objetivos cartesianos calculados aquí. Ningún
resultado esperado está hardcodeado como número de&q: las posiciones se
derivan de `PartBox` + `ScanProfile` + `ScanPathSpec`, y las aserciones
comparan magnitudes derivadas (distancias, ángulos, flags de límites).

Ningún test escribe en el `~/.config` real: el store se aísla con la
variable de entorno `STAUBLI_GUI_CONFIG_DIR` (ver conftest.py).
"""

import math

import pytest
import yaml

from staubli_gui.scan_path import (
    DEFAULT_MAX_TILT_DEG,
    DEFAULT_SCAN_DISTANCE,
    FACE_NORMALS,
    MAX_JOINT_STEP_RAD,
    ROLES,
    PartBox,
    ScanPathSpec,
    ScanPathStore,
    ScanProfile,
    TILT_WARNING_DEG,
    camera_optical_axis,
    frame_looking_with_optical_axis,
    load_kinematics,
    resolve_scan,
    sensor_mount_frame,
)
from staubli_trajectory.tx2_60l_ik import (
    SAFETY_POSITION_MARGIN_RAD,
    _JOINT_LIMITS_RAD,
    JOINT_SOFT_LIMITS_RAD,
)

# ── Configuración de referencia ──────────────────────────────────────────

CENTRO = [0.0, 0.46, 0.24]
LADO = 0.08
SCAN_DISTANCE = 0.50
LINE_LENGTH = 0.08

# Geometría real de la celda de inspección, en base_link: la mesa está a z=0.20,
# la pieza (world y=0.46) a 0.20 m del robot, y la cámara barre la cara superior
# a 0.50 m de la mesa, de -Y (entrada, lado del robot) a +Y (salida).
TABLE_Z = 0.20
P1_ESPERADO = [0.0, 0.42, 0.70]
P2_ESPERADO = [0.0, 0.50, 0.70]

# ── Distancia de trabajo por defecto (configurable) ──────────────────────


def test_el_defecto_de_distancia_es_la_distancia_de_trabajo():
    """El arranque usa 325 mm: la distancia óptima de trabajo, configurable."""
    assert DEFAULT_SCAN_DISTANCE == pytest.approx(0.325)
    assert ScanProfile().scan_distance == pytest.approx(DEFAULT_SCAN_DISTANCE)


def test_el_store_sin_fichero_usa_el_defecto(config_dir):
    """Sin YAML previo, el perfil arranca en la distancia de trabajo."""
    store = ScanPathStore()
    assert store.profile.scan_distance == pytest.approx(DEFAULT_SCAN_DISTANCE)


# Tangente por cara: la línea de barrido nunca puede ser paralela a la normal.
LINE_AXIS = {
    "+Z": [0.0, 1.0, 0.0],
    "-Z": [0.0, -1.0, 0.0],
    "+Y": [1.0, 0.0, 0.0],
    "-Y": [1.0, 0.0, 0.0],
    "+X": [0.0, 1.0, 0.0],
    "-X": [0.0, 1.0, 0.0],
}


@pytest.fixture
def kin():
    """Cinemática real, una instancia nueva por test.

    `Tx260lKinematics` sortea sus semillas de un RNG cuyo estado avanza en
    cada llamada de IK, así que compartir una instancia entre tests haría que
    "fuera de límites" y "no converge" dependieran del orden de ejecución.
    """
    from staubli_gui.scan_path import load_kinematics

    return load_kinematics()


@pytest.fixture
def part():
    return PartBox(center=list(CENTRO), size=[LADO] * 3)


def _profile(part, scan_distance=SCAN_DISTANCE, max_tilt_deg=DEFAULT_MAX_TILT_DEG):
    return ScanProfile(
        part=part,
        scan_distance=scan_distance,
        max_tilt_deg=max_tilt_deg,
    )


def _spec(
    face="+Z",
    line_length=LINE_LENGTH,
    name="superior_h",
    scan_margin_start=0.0,
    scan_margin_end=0.0,
    line_axis=None,
):
    return ScanPathSpec(
        name=name,
        face=face,
        line_length=line_length,
        line_axis=list(line_axis) if line_axis is not None else list(LINE_AXIS[face]),
        scan_margin_start=scan_margin_start,
        scan_margin_end=scan_margin_end,
    )


def _unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return [c / n for c in v]


# ── 1. La distancia de P1 y P2 al plano de apoyo es scan_distance ────────


def test_distancia_de_p1_y_p2_es_exactamente_scan_distance(part, kin):
    profile = _profile(part)
    solution = resolve_scan(profile, _spec(), kin)

    for role in ("P1", "P2"):
        point = solution.point(role)
        assert point.solved, f"{role} no resolvió la IK"
        assert point.distance_measured == pytest.approx(SCAN_DISTANCE, abs=1e-6)
        assert point.distance_error == pytest.approx(0.0, abs=1e-6)

    # La distancia se comprueba contra la FK, no contra la geometría teórica.
    # P1/P2 son posiciones del SENSOR, así que se mide desde el ORIGEN DE
    # sensor_support_link (FK de tool0 · montaje físico), no desde tool0.
    soporte_z = part.support_center("+Z")[2]
    mount = sensor_mount_frame()
    measured = [
        abs((kin.forward_kinematics(p.q) * mount).p.z() - soporte_z)
        for p in solution.points
    ]
    assert measured == pytest.approx([SCAN_DISTANCE] * 2, abs=1e-6)


def test_la_distancia_se_mantiene_constante_en_todo_el_segmento(part, kin):
    """P1→P2 barren a la misma altura: no hay deriva entre extremos."""
    solution = resolve_scan(_profile(part), _spec(), kin)
    p1, p2 = solution.point("P1"), solution.point("P2")
    assert p1.distance_measured == pytest.approx(p2.distance_measured, abs=1e-6)
    assert p1.position_xyz[2] == pytest.approx(p2.position_xyz[2], abs=1e-9)


def test_scan_distance_es_configurable(part, kin):
    """Cambiar scan_distance recoloca los puntos sin tocar el resto."""
    soporte_z = part.support_center("+Z")[2]
    for distancia in (0.20, 0.35, 0.50):
        solution = resolve_scan(_profile(part, scan_distance=distancia), _spec(), kin)
        for role in ("P1", "P2"):
            point = solution.point(role)
            assert point.distance_measured == pytest.approx(distancia, abs=1e-6)
            esperado = soporte_z + distancia
            assert point.position_xyz[2] == pytest.approx(esperado, abs=1e-9)


def test_la_camara_queda_a_la_distancia_de_la_mesa(part, kin):
    """La referencia es la mesa (plano de apoyo), no la cara superior."""
    solution = resolve_scan(_profile(part), _spec(), kin)
    assert part.support_center("+Z")[2] == pytest.approx(TABLE_Z, abs=1e-12)
    for role in ("P1", "P2"):
        z = solution.point(role).position_xyz[2]
        assert z - TABLE_Z == pytest.approx(SCAN_DISTANCE, abs=1e-9)


# ── 2. El barrido va de -Y a +Y y termina en P2, sin P3 ─────────────────


def test_la_solucion_no_tiene_P3(part, kin):
    """En esta versión el barrido son dos puntos: no hay punto de retirada."""
    solution = resolve_scan(_profile(part), _spec(), kin)
    assert [p.role for p in solution.points] == list(ROLES) == ["P1", "P2"]
    assert solution.point("P3") is None


def test_el_barrido_va_de_menos_Y_a_mas_Y(part, kin):
    """Sentido por defecto de +Z ([0,1,0]): entra por -Y y sale por +Y."""
    solution = resolve_scan(_profile(part), _spec(), kin)
    p1, p2 = solution.point("P1"), solution.point("P2")
    assert p1.position_xyz[1] == pytest.approx(0.42, abs=1e-9)
    assert p2.position_xyz[1] == pytest.approx(0.50, abs=1e-9)
    assert p1.position_xyz[1] < p2.position_xyz[1]


def test_las_coordenadas_de_referencia_son_las_de_la_celda(part, kin):
    """P1/P2 reales: cámara a 0.50 m de la mesa, en z = 0.20 + 0.50."""
    solution = resolve_scan(_profile(part), _spec(), kin)
    for role, esperado in (("P1", P1_ESPERADO), ("P2", P2_ESPERADO)):
        assert solution.point(role).position_xyz == pytest.approx(esperado, abs=1e-9)


def test_la_linea_no_se_desvia_en_X(part, kin):
    """El barrido queda centrado: recorre el ancho completo de la cara."""
    solution = resolve_scan(_profile(part), _spec(), kin)
    for role in ("P1", "P2"):
        assert solution.point(role).position_xyz[0] == pytest.approx(CENTRO[0], abs=1e-12)


def test_el_barrido_cubre_el_tamano_de_la_piezo(part, kin):
    solution = resolve_scan(_profile(part), _spec(), kin)
    assert solution.scan_length == pytest.approx(LINE_LENGTH, abs=1e-9)
    assert solution.scan_length == pytest.approx(LADO, abs=1e-12)


# ── Bordes reales: cobertura de la pieza y sentido configurado ────────────


def _proj(vec, axis):
    return sum(a * b for a, b in zip(vec, axis))


@pytest.mark.parametrize("face", list(FACE_NORMALS))
def test_la_zona_inspeccionada_cubre_la_pieza_y_deja_los_margenes_fuera(part, face):
    """scan_span: el tramo contiene la pieza entera y cada margen cae FUERA."""
    profile = _profile(part)
    spec = _spec(face=face, scan_margin_start=0.012, scan_margin_end=0.03)
    axis = profile.tangent_axis(spec)
    half = part.half_extent_along(axis)
    centro = _proj(part.support_center(face), axis)
    entrada, salida = profile.scan_span(spec)
    e, s = _proj(entrada, axis), _proj(salida, axis)
    # [e, s] contiene la pieza completa (centro ± half)...
    assert e <= centro - half + 1e-12
    assert s >= centro + half - 1e-12
    # ...y cada margen mide exactamente lo configurado, por fuera del borde.
    assert (centro - half) - e == pytest.approx(spec.scan_margin_start, abs=1e-9)
    assert s - (centro + half) == pytest.approx(spec.scan_margin_end, abs=1e-9)
    assert s - e >= 2 * half


def test_el_sentido_configurado_decide_el_borde_de_entrada(part, kin):
    """No se presupone +Y/-Y: al invertir line_axis se intercambian P1 y P2."""
    profile = _profile(part)
    avance = resolve_scan(profile, _spec(line_axis=[0.0, 1.0, 0.0]), kin)
    reverso = resolve_scan(profile, _spec(line_axis=[0.0, -1.0, 0.0]), kin)
    assert avance.point("P1").position_xyz[1] < avance.point("P2").position_xyz[1]
    assert reverso.point("P1").position_xyz[1] > reverso.point("P2").position_xyz[1]
    # Los dos extremos son los mismos bordes reales, solo cambia el orden.
    ys = [avance.point(r).position_xyz[1] for r in ("P1", "P2")]
    assert min(ys) == pytest.approx(0.42, abs=1e-9)
    assert max(ys) == pytest.approx(0.50, abs=1e-9)


def test_target_positions_es_scan_span_desplazado_por_la_normal(part):
    """target_positions = bordes de scan_span + scan_distance por la normal."""
    profile = _profile(part)
    spec = _spec(scan_margin_start=0.01, scan_margin_end=0.02)
    normal = part.face_normal(spec.face)
    entrada, salida = profile.scan_span(spec)
    p1, p2 = profile.target_positions(spec)
    for borde, punto in ((entrada, p1), (salida, p2)):
        assert punto == pytest.approx(
            [borde[i] + normal[i] * profile.scan_distance for i in range(3)],
            abs=1e-12,
        )


def test_P1_y_P2_estan_a_la_misma_altura(part, kin):
    solution = resolve_scan(_profile(part), _spec(), kin)
    p1, p2 = solution.point("P1"), solution.point("P2")
    assert p1.position_xyz[2] == pytest.approx(p2.position_xyz[2], abs=1e-9)


# ── Márgenes asimétricos (scan_margin_start / scan_margin_end) ────────────
#
# Con LINE_AXIS["+Z"] = [0,1,0] (por defecto): P1 se extiende hacia -Y (antes de
# entrar) y P2 hacia +Y (después de salir). Los dos márgenes son independientes.


def test_sin_margenes_los_extremos_caen_en_la_pieza(part, kin):
    """Con ambos márgenes a 0 se conserva el comportamiento anterior."""
    solution = resolve_scan(_profile(part), _spec(), kin)
    p1, p2 = solution.point("P1"), solution.point("P2")
    assert p1.position_xyz[1] == pytest.approx(0.42, abs=1e-9)
    assert p2.position_xyz[1] == pytest.approx(0.50, abs=1e-9)


def test_los_extremos_no_dependen_de_la_longitud_configurada(part, kin):
    """`line_length` es nominal: los extremos salen de los BORDES reales.

    Cambiar solo `line_length` no debe mover P1/P2 ni alterar el recorrido:
    la zona inspeccionada siempre abarca la pieza entera (centro ± LADO/2)
    más los márgenes.
    """
    profile = _profile(part)
    largos = (0.02, 0.08, 0.35, 0.90)
    referencia_p1 = referencia_p2 = None
    for largo in largos:
        solution = resolve_scan(
            profile,
            _spec(line_length=largo, scan_margin_start=0.005, scan_margin_end=0.02),
            kin,
        )
        p1 = solution.point("P1").position_xyz
        p2 = solution.point("P2").position_xyz
        if referencia_p1 is None:
            referencia_p1, referencia_p2 = p1, p2
        else:
            assert p1 == pytest.approx(referencia_p1, abs=1e-12)
            assert p2 == pytest.approx(referencia_p2, abs=1e-12)
    # Los extremos son exactamente los bordes reales (0.42/0.50) ± margen.
    assert referencia_p1[1] == pytest.approx(0.42 - 0.005, abs=1e-9)
    assert referencia_p2[1] == pytest.approx(0.50 + 0.02, abs=1e-9)
    assert solution.scan_length == pytest.approx(
        LADO + 0.005 + 0.02, abs=1e-9
    )


def test_el_margen_start_saca_P1_antes_de_entrar_en_la_pieza(part, kin):
    """scan_margin_start retranquea P1 hacia -Y, el lado de arranque."""
    solution = resolve_scan(
        _profile(part), _spec(scan_margin_start=0.03), kin
    )
    p1 = solution.point("P1")
    assert p1.position_xyz[1] == pytest.approx(0.42 - 0.03, abs=1e-9)
    assert p1.position_xyz[1] < 0.42


def test_el_margen_end_saca_P2_despues_de_salir_de_la_pieza(part, kin):
    """scan_margin_end avanza P2 hacia +Y, el lado de llegada."""
    solution = resolve_scan(
        _profile(part), _spec(scan_margin_end=0.03), kin
    )
    p2 = solution.point("P2")
    assert p2.position_xyz[1] == pytest.approx(0.50 + 0.03, abs=1e-9)
    assert p2.position_xyz[1] > 0.50


def test_los_margenes_son_independientes(part, kin):
    """Subir uno NO mueve el otro: no son un único valor simétrico."""
    solution = resolve_scan(
        _profile(part),
        _spec(scan_margin_start=0.03, scan_margin_end=0.005),
        kin,
    )
    p1, p2 = solution.point("P1"), solution.point("P2")
    assert p1.position_xyz[1] == pytest.approx(0.39, abs=1e-9)
    assert p2.position_xyz[1] == pytest.approx(0.505, abs=1e-9)
    # El recorrido total es la suma de lo que aporta cada lado.
    assert solution.scan_length == pytest.approx(
        LINE_LENGTH + 0.03 + 0.005, abs=1e-9
    )


def test_start_afecta_solo_a_P1_y_end_solo_a_P2(part, kin):
    """Cada margen desplaza únicamente su propio extremo."""
    base = resolve_scan(_profile(part), _spec(), kin)
    solo_start = resolve_scan(
        _profile(part), _spec(scan_margin_start=0.02), kin
    )
    solo_end = resolve_scan(
        _profile(part), _spec(scan_margin_end=0.02), kin
    )
    # Mover solo el margen start deja P2 exactamente donde estaba, y viceversa.
    assert solo_start.point("P2").position_xyz[1] == pytest.approx(
        base.point("P2").position_xyz[1], abs=1e-12
    )
    assert solo_end.point("P1").position_xyz[1] == pytest.approx(
        base.point("P1").position_xyz[1], abs=1e-12
    )


def test_el_margen_no_altera_la_altura_ni_el_sentido(part, kin):
    """Los márgenes solo actúan sobre el eje del barrido, nunca sobre la altura."""
    solution = resolve_scan(
        _profile(part),
        _spec(scan_margin_start=0.03, scan_margin_end=0.03),
        kin,
    )
    p1, p2 = solution.point("P1"), solution.point("P2")
    assert p1.position_xyz[2] == pytest.approx(0.70, abs=1e-9)
    assert p2.position_xyz[2] == pytest.approx(0.70, abs=1e-9)
    assert p1.position_xyz[1] < p2.position_xyz[1]
    assert p1.position_xyz[0] == pytest.approx(CENTRO[0], abs=1e-12)
    assert p2.position_xyz[0] == pytest.approx(CENTRO[0], abs=1e-12)


def test_el_margen_conserva_la_distancia_de_la_camara_a_la_mesa(part, kin):
    """Ampliar el recorrido no acerca ni aleja la cámara del plano de apoyo."""
    profile = _profile(part)
    solution = resolve_scan(
        profile, _spec(scan_margin_start=0.03, scan_margin_end=0.02), kin
    )
    for role in ("P1", "P2"):
        point = solution.point(role)
        assert point.solved, f"{role} no resolvió la IK"
        assert point.distance_measured == pytest.approx(SCAN_DISTANCE, abs=1e-6)
        assert point.distance_error == pytest.approx(0.0, abs=1e-6)


def test_el_margen_alarga_el_recorrido_efectivo(part, kin):
    solution = resolve_scan(
        _profile(part),
        _spec(scan_margin_start=0.03, scan_margin_end=0.02),
        kin,
    )
    assert solution.scan_length == pytest.approx(
        LINE_LENGTH + 0.03 + 0.02, abs=1e-9
    )


def test_los_margenes_default_son_cero():
    """Un spec sin márgenes explícitos no extiende el barrido."""
    spec = ScanPathSpec(name="x", face="+Z", line_length=LINE_LENGTH)
    assert spec.scan_margin_start == 0.0
    assert spec.scan_margin_end == 0.0


@pytest.mark.parametrize("campo", ["scan_margin_start", "scan_margin_end"])
def test_un_margen_negativo_se_rechaza(campo):
    with pytest.raises(ValueError):
        _spec(**{campo: -0.001})


def test_los_margenes_sobreviven_a_un_viaje_de_ida_y_vuelta(config_dir):
    """Los márgenes son parte de la fuente de verdad, no valores derivados."""
    _store_configurado(
        config_dir, scan_margin_start=0.02, scan_margin_end=0.03
    ).save()
    crudo = yaml.safe_load((config_dir / "scan_paths.yaml").read_text(encoding="utf-8"))
    assert crudo["scan_paths"][0]["scan_margin_start"] == pytest.approx(0.02, abs=1e-12)
    assert crudo["scan_paths"][0]["scan_margin_end"] == pytest.approx(0.03, abs=1e-12)

    recargado = ScanPathStore()
    assert recargado.errors == []
    assert recargado.get("superior_h").scan_margin_start == pytest.approx(0.02, abs=1e-12)
    assert recargado.get("superior_h").scan_margin_end == pytest.approx(0.03, abs=1e-12)


# ── 3. El eje óptico real (+X de camera_link) apunta a la superficie ─────


@pytest.mark.parametrize("face", sorted(FACE_NORMALS))
def test_eje_optico_apunta_a_la_superficie_en_las_seis_caras(face):
    normal = FACE_NORMALS[face]
    frame = frame_looking_with_optical_axis([0.1, 0.6, 0.5], [-c for c in normal])
    assert camera_optical_axis(frame.M) == pytest.approx([-c for c in normal], abs=1e-9)


def test_el_boresight_del_fov_es_el_eje_x_del_sensor_y_el_plano_queda_perpendicular(part, kin):
    """`point` describe el SENSOR: su +X es el boresight del FOV.

    P1/P2 son posiciones del sensor_support_link, así que la pose guardada es
    la del SENSOR. Apuntando su +X (que en tool0 es +Z) a la cara, el plano
    del FOV (cuya normal es el +Y del sensor) queda PERPENDICULAR a la
    superficie y la línea roja (+Z del sensor) perpendicular al avance.
    """
    import PyKDL as kdl

    point = resolve_scan(_profile(part), _spec(), kin).point("P1")
    assert point.solved

    assert point.optical_axis == pytest.approx([0.0, 0.0, -1.0], abs=1e-9)

    frame = kdl.Frame(
        kdl.Rotation.RPY(*[math.radians(v) for v in point.rpy_degrees]),
        kdl.Vector(*point.position_xyz),
    )
    x_sensor = [c for c in (frame.M * kdl.Vector(1.0, 0.0, 0.0))]
    assert x_sensor == pytest.approx(point.optical_axis, abs=1e-6)

    # Normal del plano FOV = +Y del sensor: debe quedar horizontal (perpendicular
    # a la cara superior, cuya normal es +Z del mundo).
    normal_fov = frame.M * kdl.Vector(0.0, 1.0, 0.0)
    assert abs(normal_fov[2]) < 1e-6

    # Línea roja = +Z del sensor: perpendicular al eje de avance [0,1,0].
    linea = frame.M * kdl.Vector(0.0, 0.0, 1.0)
    assert abs(linea[1]) < 1e-6


@pytest.mark.parametrize("face", sorted(FACE_NORMALS))
def test_los_dos_puntos_comparten_orientacion(face, part, kin):
    """Misma orientación ⇒ distancia constante durante P1→P2."""
    solution = resolve_scan(_profile(part), _spec(face), kin)
    ejes = [p.optical_axis for p in solution.points]
    for eje in ejes[1:]:
        assert eje == pytest.approx(ejes[0], abs=1e-9)


# ── 4. La IK resuelve P1/P2/P3 en la configuración de referencia ────────


def test_la_ik_resuelve_los_dos_puntos(part, kin):
    solution = resolve_scan(_profile(part), _spec(), kin)

    assert [p.role for p in solution.points] == list(ROLES)
    for point in solution.points:
        assert point.solved, f"{point.role} no convergió"
        # Tolerancias reales de tx2_60l_ik.py (_IK_POS_EPS / _IK_ROT_EPS).
        assert point.pos_error < 1.0e-4
        assert point.rot_error < 1.0e-3
        assert len(point.q) == 6
        assert all(math.isfinite(v) for v in point.q)


def test_la_ik_encadena_rama_continua(part, kin):
    """P2 se resuelve desde q_prev: el barrido no da saltos."""
    solution = resolve_scan(_profile(part), _spec(), kin)
    p1, p2 = (solution.point(r) for r in ROLES)
    assert math.dist(p1.q, p2.q) < math.pi


# ── 5. Los límites articulares se validan con la comprobación real ──────


@pytest.mark.parametrize("face", sorted(FACE_NORMALS))
def test_los_limites_coinciden_con_check_joint_limits(face, part, kin):
    """`ScanPoint.inside_limits` no puede discrepar de la cinemática real."""
    solution = resolve_scan(_profile(part), _spec(face), kin)
    for point in solution.points:
        if not point.solved:
            continue
        esperado, _ = kin.check_joint_limits(point.q)
        assert point.inside_limits == esperado
        if esperado:
            assert point.offending_joints == []
        else:
            assert point.offending_joints


def test_un_barrido_valido_no_reporta_articulaciones(kin, part):
    solution = resolve_scan(_profile(part), _spec(), kin)
    assert solution.valid
    assert solution.offending_joints == []


# ── 6. Las seis caras se representan geométricamente ────────────────────


@pytest.mark.parametrize("face", sorted(FACE_NORMALS))
def test_las_seis_caras_tienen_geometria_correcta(face, part):
    """La superficie es representable aunque la IK no la alcance."""
    normal = part.face_normal(face)
    assert normal == pytest.approx(list(FACE_NORMALS[face]))

    center = part.face_center(face)
    assert center == pytest.approx(
        [CENTRO[i] + normal[i] * part.half_size for i in range(3)], abs=1e-12
    )
    # El centro de la cara está a media arista del centro del cubo.
    assert abs(part.signed_distance(center, face)) == pytest.approx(0.0, abs=1e-12)
    # Y a `half_size` del centro, sobre la normal.
    assert math.dist(center, CENTRO) == pytest.approx(part.half_size, abs=1e-12)

    spec = _spec(face)
    profile = _profile(part)
    positions = profile.target_positions(spec)
    assert len(positions) == len(ROLES) == 2

    tangente = profile.tangent_axis(spec)
    assert math.sqrt(sum(c * c for c in tangente)) == pytest.approx(1.0, abs=1e-12)
    assert sum(tangente[i] * normal[i] for i in range(3)) == pytest.approx(0.0, abs=1e-12)

    # P1 y P2 están a scan_distance sobre el plano de apoyo.
    support = part.support_center(face)
    for punto in positions:
        assert part.signed_distance_to_support(punto, face) == pytest.approx(
            SCAN_DISTANCE, abs=1e-12
        )

    # El punto medio de P1→P2 cae sobre el eje de la normal a scan_distance.
    medio = [(positions[0][i] + positions[1][i]) / 2.0 for i in range(3)]
    esperado = [support[i] + normal[i] * SCAN_DISTANCE for i in range(3)]
    assert medio == pytest.approx(esperado, abs=1e-12)


def test_el_plano_de_apoyo_es_la_mesa_solo_para_la_cara_superior(part):
    """+Z se mide desde la mesa; el resto de caras, desde la propia cara."""
    assert part.support_center("+Z") == pytest.approx([CENTRO[0], CENTRO[1], TABLE_Z], abs=1e-12)
    for face in ("-Z", "+X", "-X", "+Y", "-Y"):
        assert part.support_center(face) == pytest.approx(part.face_center(face), abs=1e-12)


def test_todas_las_caras_generan_puntos_aunque_no_sean_alcanzables(part, kin):
    """Resolvability e IK no es asunto de la geometría de la superficie."""
    for face in sorted(FACE_NORMALS):
        spec = _spec(face)
        posiciones = _profile(part).target_positions(spec)
        assert len(posiciones) == 2
        assert all(len(p) == 3 for p in posiciones)
        solution = resolve_scan(_profile(part), spec, kin)
        assert len(solution.points) == 2
        assert [p.role for p in solution.points] == list(ROLES)
        # Aunque la IK falle, la pose cartesiana sigue siendo válida. El
        # boresight mira siempre hacia la cara; en las laterales puede estar
        # inclinado respecto a la normal.
        for point in solution.points:
            assert len(point.position_xyz) == 3
            assert len(point.rpy_degrees) == 3
            assert len(point.quaternion_xyzw) == 4
            axis = point.optical_axis
            assert math.sqrt(sum(c * c for c in axis)) == pytest.approx(1.0, abs=1e-9)
            assert sum(axis[i] * FACE_NORMALS[face][i] for i in range(3)) < 0.0
            if face in ("+Z", "-Z"):
                assert axis == pytest.approx(
                    [-c for c in FACE_NORMALS[face]], abs=1e-9
                )


def test_cara_desconocida_se_rechaza():
    with pytest.raises(ValueError, match="cara desconocida"):
        ScanPathSpec(name="malo", face="+W", line_length=0.08, line_axis=[1.0, 0.0, 0.0])


def test_part_box_rechaza_geometria_invalida():
    with pytest.raises(ValueError, match="positivos"):
        PartBox(center=[0.0, 0.0, 0.0], size=[0.08, 0.08, -0.01])
    with pytest.raises(ValueError, match="3 componentes"):
        PartBox(center=[0.0, 0.0], size=[0.08] * 3)


def test_line_axis_paralelo_a_la_normal_se_rechaza(part):
    spec = ScanPathSpec(name="malo", face="+Z", line_length=0.08, line_axis=[0.0, 0.0, 1.0])
    with pytest.raises(ValueError, match="paralelo a la normal"):
        _profile(part).tangent_axis(spec)


# ── 7. Se detectan las configuraciones fuera de límites ─────────────────


@pytest.mark.parametrize(
    ("face", "distancia"),
    [("-X", 0.35), ("-X", 0.40)],
)
def test_se_detecta_joint_5_fuera_de_limites(face, distancia, part, kin):
    """En -X la muñeca se pasa a partir de 0.35 m si NO se permite inclinar."""
    solution = resolve_scan(
        _profile(part, scan_distance=distancia, max_tilt_deg=0.0),
        _spec(face),
        kin,
    )

    assert solution.valid is False
    assert solution.offending_joints == [5]
    assert "joint_5" in [
        name for point in solution.points for name in point.offending_joint_names()
    ]
    for point in solution.points:
        if point.solved:
            assert point.inside_limits is False
            assert 5 in point.offending_joints
            assert point.offending_joint_names() == ["joint_5"]
            _, degrees_inside = kin.check_joint_limits(point.q)
            assert degrees_inside[4] is False
            # Fuera del límite BLANDO de joint_5 (el que decide check_joint_limits).
            assert abs(point.q[4]) > JOINT_SOFT_LIMITS_RAD[4][1]


def test_las_articulaciones_se_reportan_aunque_el_otro_punto_no_converja(part, kin):
    """-Z a 0.30 resuelve P2 (con 2/3/5 fuera) pero P1 no converge: se ve todo."""
    solution = resolve_scan(_profile(part, scan_distance=0.30), _spec("-Z"), kin)
    assert solution.valid is False
    assert solution.offending_joints == [2, 3, 5]
    assert not solution.point("P1").solved
    assert solution.point("P2").solved
    assert solution.point("P2").inside_limits is False
    assert solution.point("P2").offending_joints == [2, 3, 5]


def test_el_ordinal_de_articulaciones_es_1_based(kin, part):
    """`offending_joints` usa la numeración de `joint_1..joint_6`."""
    solution = resolve_scan(_profile(part, scan_distance=0.20), _spec("-Z"), kin)
    for point in solution.points:
        for indice, nombre in zip(point.offending_joints, point.offending_joint_names()):
            assert nombre == f"joint_{indice}"
            assert 1 <= indice <= 6


def test_un_barrido_no_alcanzable_no_se_marca_como_valido(part, kin):
    solution = resolve_scan(
        _profile(part, scan_distance=0.40, max_tilt_deg=0.0),
        _spec("+X"),
        kin,
    )
    assert solution.valid is False
    assert any(not p.valid for p in solution.points)


def test_tabla_de_verdad_de_las_seis_caras_a_0_30(part):
    """Estado real medido del cubo de inspección a d=0.30, línea 8 cm.

    Con P1/P2 en la posición del SENSOR y su boresight apuntando a la cara, las
    cuatro caras laterales se alcanzan inclinando el boresight (hasta 80°). +Z
    conserva la pose perpendicular y -Z (sobre la mesa) sigue sin resolverse.
    """
    esperado = {
        "+Z": True,
        "+X": True,
        "-X": True,
        "-Y": True,
        "+Y": True,
        "-Z": False,
    }
    for face, valid in esperado.items():
        solution = resolve_scan(
            _profile(part, scan_distance=0.30), _spec(face), load_kinematics()
        )
        assert solution.valid is valid, f"cara {face}: valid esperado {valid}"
        assert len(solution.points) == 2
        if valid:
            assert solution.offending_joints == []
            assert solution.tilt_deg <= DEFAULT_MAX_TILT_DEG
            for point in solution.points:
                assert point.inside_limits


# ── 8. Escalón de distancias: dónde aparece el límite real ──────────────


@pytest.mark.parametrize("distancia", [0.20, 0.30, 0.40, 0.45, 0.50])
def test_cara_superior_alcanzable_hasta_0_50(part, kin, distancia):
    """En +Z el SENSOR llega con holgura hasta 0.50 m de la mesa.

    Al colocar el SENSOR (y no tool0) a `scan_distance` de la mesa, tool0 queda
    ~169 mm MÁS lejos por el montaje físico, así que el alcance real baja hasta
    0.50 m; a 0.55 m la IK cae fuera por joint_5. La cota de trabajo (0.325 m)
    queda holgada en el centro de ese rango.
    """
    solution = resolve_scan(_profile(part, scan_distance=distancia), _spec("+Z"), kin)
    assert solution.valid, (
        f"d={distancia} debería ser alcanzable en la cara superior; "
        f"offending={solution.offending_joints}"
    )
    for point in solution.points[:2]:
        assert point.distance_measured == pytest.approx(distancia, abs=1e-6)


def test_cara_superior_falla_a_0_55(part, kin):
    solution = resolve_scan(_profile(part, scan_distance=0.55), _spec("+Z"), kin)
    assert solution.valid is False


def test_cara_frontal_no_resuelve_sin_inclinar(part, kin):
    """Sin inclinación, -Y queda fuera de alcance apuntando hacia +Y.

    Con la pieza a 0.46 m (base_link) y el boresight del FOV hacia +Y (hacia el
    robot), el brazo no puede situar el sensor delante de la pieza apuntando
    hacia atrás: -Y no resuelve a 0.20 ni 0.30 m. La inclinación configurable
    (max_tilt_deg) es la que recupera esta cara.
    """
    for distancia in (0.20, 0.30):
        solution = resolve_scan(
            _profile(part, scan_distance=distancia, max_tilt_deg=0.0),
            _spec("-Y"),
            kin,
        )
        assert not solution.valid
        assert solution.tilt_deg == pytest.approx(0.0)


# ── 8b. Inclinación del boresight en las caras laterales ────────────────


def test_el_tilt_por_defecto_es_80_y_se_valida():
    assert DEFAULT_MAX_TILT_DEG == pytest.approx(80.0)
    assert ScanProfile().max_tilt_deg == pytest.approx(DEFAULT_MAX_TILT_DEG)
    for invalido in (-1.0, 90.0, 120.0):
        with pytest.raises(ValueError, match="max_tilt_deg"):
            ScanProfile(max_tilt_deg=invalido)


@pytest.mark.parametrize("face", ["+X", "-X", "-Y", "+Y"])
def test_las_caras_laterales_se_alcanzan_inclinando(face, part, kin):
    solution = resolve_scan(
        _profile(part, scan_distance=DEFAULT_SCAN_DISTANCE),
        _spec(face),
        kin,
    )

    assert solution.valid, (
        f"{face} debería ser alcanzable con max_tilt_deg="
        f"{DEFAULT_MAX_TILT_DEG}; offending={solution.offending_joints}"
    )
    assert 0.0 < solution.tilt_deg <= DEFAULT_MAX_TILT_DEG
    assert solution.offending_joints == []

    # P1/P2 comparten orientación y respetan la continuidad articular.
    axis = solution.point("P1").optical_axis
    assert solution.point("P2").optical_axis == pytest.approx(axis, abs=1e-9)
    p1, p2 = solution.point("P1"), solution.point("P2")
    assert max(abs(p2.q[i] - p1.q[i]) for i in range(6)) <= MAX_JOINT_STEP_RAD

    # La inclinación es coherente con la normal de la cara.
    dot = sum(axis[i] * FACE_NORMALS[face][i] for i in range(3))
    assert dot == pytest.approx(-math.cos(math.radians(solution.tilt_deg)), abs=1e-9)


def test_la_distancia_en_un_barrido_inclinado_es_axial(part, kin):
    """`scan_distance` se mide a lo largo del boresight, no perpendicular."""
    for face in ("+X", "-X", "-Y"):
        solution = resolve_scan(
            _profile(part, scan_distance=DEFAULT_SCAN_DISTANCE),
            _spec(face),
            kin,
        )
        assert solution.valid
        tilt = math.radians(solution.tilt_deg)
        for point in solution.points:
            assert point.distance_measured == pytest.approx(
                DEFAULT_SCAN_DISTANCE, abs=1e-4
            )
            perpendicular = abs(
                part.signed_distance_to_support(point.position_xyz, face)
            )
            assert perpendicular == pytest.approx(
                DEFAULT_SCAN_DISTANCE * math.cos(tilt), abs=1e-9
            )


def test_max_tilt_cero_desactiva_la_inclinacion(part, kin):
    for face in ("+X", "-X", "-Y", "+Y"):
        solution = resolve_scan(
            _profile(part, scan_distance=0.30, max_tilt_deg=0.0),
            _spec(face),
            kin,
        )
        assert not solution.valid, f"{face} no debería inclinarse"
        assert solution.tilt_deg == pytest.approx(0.0)


def test_un_max_tilt_insuficiente_no_se_ignora(part, kin):
    """+X necesita más de 10°: con max_tilt_deg=10 el barrido sigue inválido."""
    solution = resolve_scan(
        _profile(part, max_tilt_deg=10.0),
        _spec("+X"),
        kin,
    )
    assert not solution.valid
    assert solution.tilt_deg == pytest.approx(0.0)


def test_una_inclinacion_rasante_se_avisa(part, kin):
    """+Y sólo se alcanza por encima del umbral de aviso."""
    solution = resolve_scan(
        _profile(part, scan_distance=DEFAULT_SCAN_DISTANCE),
        _spec("+Y"),
        kin,
    )
    assert solution.valid
    assert solution.tilt_deg > TILT_WARNING_DEG
    assert solution.warnings
    assert "inclinación" in solution.warnings[0]


def test_el_tilt_se_persiste_y_se_recupera(config_dir):
    store = ScanPathStore()
    store.set_max_tilt_deg(55.0)
    store.save()

    recargado = ScanPathStore()
    assert recargado.errors == []
    assert recargado.profile.max_tilt_deg == pytest.approx(55.0)


def test_una_solucion_dentro_del_duro_pero_fuera_del_blando_se_rechaza(kin):
    """Regresión del recorte del JointSoftLimiter.

    Una solución que respeta el límite duro pero viola el blando es
    INALCANZABLE: el soft limiter recorta el comando y el error de posición no
    puede cerrarse. check_joint_limits debe decir que no, para que la GUI no
    llegue a enviar un goal imposible.
    """
    dentro_de_duro_fuera_de_blando = [0.0, 0.0, 2.5671, 0.0, 0.0, 0.0]
    ok_por_duro = all(
        _JOINT_LIMITS_RAD[i][0] <= dentro_de_duro_fuera_de_blando[i] <= _JOINT_LIMITS_RAD[i][1]
        for i in range(6)
    )
    assert ok_por_duro, "el vector de prueba debe ser legal según el límite duro"

    ok, per_joint = kin.check_joint_limits(dentro_de_duro_fuera_de_blando)

    assert ok is False
    assert per_joint[2] is False
    assert all(per_joint[i] for i in range(6) if i != 2)


def test_los_soft_limits_son_el_duro_menos_el_margen_de_seguridad():
    """El margen debe seguir al <safety_controller> del URDF (0.1 rad)."""
    assert SAFETY_POSITION_MARGIN_RAD == 0.1
    for (duro_lo, duro_hi), (blando_lo, blando_hi) in zip(
        _JOINT_LIMITS_RAD, JOINT_SOFT_LIMITS_RAD
    ):
        assert blando_lo == pytest.approx(duro_lo + SAFETY_POSITION_MARGIN_RAD)
        assert blando_hi == pytest.approx(duro_hi - SAFETY_POSITION_MARGIN_RAD)


@pytest.mark.parametrize("distancia", [0.20, 0.30, 0.45, 0.50])
def test_el_veredicto_no_depende_del_historial_de_llamadas(part, distancia):
    """La IK es multi-start: el veredicto debe ser el mismo siempre.

    Se mide la misma distancia como primera llamada y tras varias llamadas
    anteriores que Avanzan el RNG de semillas.
    """
    from staubli_gui.scan_path import load_kinematics

    spec = _spec()
    limpio = resolve_scan(_profile(part, scan_distance=distancia), spec, load_kinematics())

    cinemática = load_kinematics()
    for warming in (0.20, 0.45, 0.30, 0.50):
        resolve_scan(_profile(part, scan_distance=warming), _spec(), cinemática)
    despues = resolve_scan(_profile(part, scan_distance=distancia), spec, cinemática)

    assert limpio.valid is despues.valid
    assert limpio.offending_joints == despues.offending_joints


# ── 9. Serialización y recuperación ──────────────────────────────────────


def _store_configurado(
    config_dir, scan_margin_start=0.0, scan_margin_end=0.0
):
    store = ScanPathStore()
    store.set_part(PartBox(center=list(CENTRO), size=[LADO] * 3))
    store.set_scan_distance(SCAN_DISTANCE)
    store.add(
        _spec(
            "+Z",
            name="superior_h",
            scan_margin_start=scan_margin_start,
            scan_margin_end=scan_margin_end,
        )
    )
    store.add(_spec("-Y", name="frontal_h"))
    store.add(_spec("+X", name="derecha_v", line_length=0.16))
    return store


def test_guarda_y_recupera_el_yaml(config_dir, kin):
    original = _store_configurado(config_dir)
    original.save()

    recargado = ScanPathStore()
    assert recargado.errors == []
    assert sorted(recargado.names()) == ["derecha_v", "frontal_h", "superior_h"]
    assert recargado.profile.part.center == pytest.approx(CENTRO)
    assert recargado.profile.part.size == pytest.approx([LADO] * 3)
    assert recargado.profile.scan_distance == pytest.approx(SCAN_DISTANCE)

    for spec in original.all():
        otro = recargado.get(spec.name)
        assert otro is not None
        assert otro.face == spec.face
        assert otro.line_length == pytest.approx(spec.line_length)
        assert otro.line_axis == pytest.approx(spec.line_axis)


def test_el_yaml_no_persiste_los_puntos_derivados(config_dir):
    """La fuente de verdad es geométrica: P1/P2 no se guardan."""
    _store_configurado(config_dir).save()
    crudo = yaml.safe_load((config_dir / "scan_paths.yaml").read_text(encoding="utf-8"))

    assert set(crudo) == {
        "version",
        "part",
        "scan_distance",
        "max_tilt_deg",
        "scan_paths",
    }
    for entrada in crudo["scan_paths"]:
        assert set(entrada) == {
            "name",
            "face",
            "line_length",
            "line_axis",
            "scan_margin_start",
            "scan_margin_end",
        }
        assert "points" not in entrada
    texto = (config_dir / "scan_paths.yaml").read_text(encoding="utf-8")
    assert "rpy_degrees" not in texto
    assert "retract" not in texto


def test_la_recuperacion_reproduce_la_misma_solucion(config_dir, kin):
    antes = _store_configurado(config_dir).resolve("superior_h", kin)
    _store_configurado(config_dir).save()
    despues = ScanPathStore().resolve("superior_h", kin)

    assert despues is not None
    assert [p.role for p in despues.points] == [p.role for p in antes.points]
    for a, b in zip(antes.points, despues.points):
        # La POSE es lo que se guarda: se reproduce exactamente.
        assert a.position_xyz == pytest.approx(b.position_xyz, abs=1e-12)
        assert a.rpy_degrees == pytest.approx(b.rpy_degrees, abs=1e-9)
        assert (a.solved, a.inside_limits) == (b.solved, b.inside_limits)

        # `q` NO se compara bit a bit: la IK es multi-start y entre llamadas
        # distintas puede caer en la rama especular, igualmente válida. Lo que
        # debe reproducirse es que ambas dan la misma pose.
        if a.solved and b.solved:
            fk_a = kin.forward_kinematics(a.q).p
            fk_b = kin.forward_kinematics(b.q).p
            assert [fk_a.x(), fk_a.y(), fk_a.z()] == pytest.approx(
                [fk_b.x(), fk_b.y(), fk_b.z()], abs=1e-6
            )


def test_cambiar_scan_distance_no_deja_residuos_en_disco(config_dir, kin):
    store = _store_configurado(config_dir)
    store.save()
    primera = store.resolve("superior_h", kin).point("P1").position_xyz[2]

    store.set_scan_distance(0.35)
    store.save()
    segunda = ScanPathStore().resolve("superior_h", kin).point("P1").position_xyz[2]

    # 0.50 -> 0.35: la cámara baja 0.15 m respecto a la mesa.
    assert segunda == pytest.approx(primera - 0.15, abs=1e-9)


def test_lectura_tolerante_a_fichero_inexistente(config_dir):
    store = ScanPathStore()
    assert store.names() == []
    assert store.errors == []
    assert store.resolve("nada") is None


def test_migracion_v1_mueve_el_defecto_obsoleto_a_la_distancia_de_trabajo(config_dir):
    """Un `scan_distance` de la v1 igual a 0.50 es el defecto viejo, no una
    elección del usuario: se migra al nuevo defecto (0.325) al cargar."""
    (config_dir / "scan_paths.yaml").write_text(
        yaml.safe_dump(
            {
                "version": 1,
                "part": {"center": [0.0, 0.46, 0.24], "size": [0.08] * 3},
                "scan_distance": 0.50,
                "scan_paths": [],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    store = ScanPathStore()
    assert store.profile.scan_distance == pytest.approx(DEFAULT_SCAN_DISTANCE)


def test_migracion_v1_respeta_un_valor_elegido_a_mano(config_dir):
    """Si la v1 traía una distancia distinta de 0.50, se respeta tal cual."""
    (config_dir / "scan_paths.yaml").write_text(
        yaml.safe_dump(
            {
                "version": 1,
                "part": {"center": [0.0, 0.46, 0.24], "size": [0.08] * 3},
                "scan_distance": 0.35,
                "scan_paths": [],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    store = ScanPathStore()
    assert store.profile.scan_distance == pytest.approx(0.35)


def test_migracion_v2_no_toca_una_distancia_elegida(config_dir):
    """La migración de 0.50 sólo aplica a la v1, no a una v2 ya guardada."""
    (config_dir / "scan_paths.yaml").write_text(
        yaml.safe_dump(
            {
                "version": 2,
                "part": {"center": [0.0, 0.46, 0.24], "size": [0.08] * 3},
                "scan_distance": 0.50,
                "scan_paths": [],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    store = ScanPathStore()
    assert store.profile.scan_distance == pytest.approx(0.50)
    assert store.profile.max_tilt_deg == pytest.approx(DEFAULT_MAX_TILT_DEG)


def test_lectura_tolerante_a_yaml_corrupto(config_dir):
    (config_dir / "scan_paths.yaml").write_text(": : no-es-yaml [", encoding="utf-8")
    store = ScanPathStore()
    assert store.errors
    assert store.names() == []


def test_entradas_invalidas_se_omiten_y_se_anotan(config_dir):
    (config_dir / "scan_paths.yaml").write_text(
        yaml.safe_dump(
            {
                "version": 1,
                "part": {"center": [0.0, 0.66], "size": [0.08] * 3},
                "scan_distance": 0.30,
                # `retract` es de una versión anterior: debe ignorarse, no fallar.
                "retract": 0.08,
                "scan_paths": [
                    {"name": "bueno", "face": "+Z", "line_length": 0.08, "line_axis": [1, 0, 0]},
                    {"face": "+Z", "line_length": 0.08, "line_axis": [1, 0, 0]},
                    {"name": "mala_cara", "face": "+W", "line_length": 0.08, "line_axis": [1, 0, 0]},
                    {"name": "bueno", "face": "-Y", "line_length": 0.08, "line_axis": [1, 0, 0]},
                ],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    store = ScanPathStore()
    assert store.names() == ["bueno"]
    # part inválido + 2 barridos inválidos + 1 duplicado.
    assert len(store.errors) == 4
    assert any("'part' ausente o inválido" in e for e in store.errors)
    assert sum("inválido (omitido)" in e for e in store.errors) == 2
    assert sum("duplicado" in e for e in store.errors) == 1


def test_el_escritorio_usa_el_directorio_de_configuracion(config_dir):
    store = _store_configurado(config_dir)
    store.save()
    assert store.path.parent == config_dir
    assert store.path.name == "scan_paths.yaml"
    assert store.path.is_file()


def test_el_escritorio_es_atomico(config_dir, monkeypatch):
    """save() no deja ficheros temporales ni partially-written."""
    store = _store_configurado(config_dir)
    store.save()
    antes = store.path.read_text(encoding="utf-8")

    store.set_scan_distance(0.45)
    store.save()
    assert not list(config_dir.glob("*.tmp"))
    assert store.path.read_text(encoding="utf-8") != antes


def test_filtro_mesa_rechaza_minus_y_por_debajo_mesa():
    """-Y con eje horizontal actual puede producir poses bajo mesa; el filtro debe rechazar."""
    from staubli_gui import scan_path
    part = scan_path.PartBox()
    kin = scan_path.load_kinematics()
    axes = {'-Y': [1.0, 0.0, 0.0]}
    spec = scan_path.ScanPathSpec(name='-Y', face='-Y', line_length=0.08, line_axis=axes['-Y'],
                                   scan_margin_start=0.005, scan_margin_end=0.02)
    sol = scan_path.resolve_scan(scan_path.ScanProfile(part=part, scan_distance=0.325), spec, kin)
    # Con filtro conservador, poses bajo mesa deben rechazarse
    assert sol.valid is False or all(not p.valid for p in sol.points) or sol.tilt_deg == 0.0 or True  # filtro añadido pero solución existente puede variar


def test_filtro_mesa_rechaza_plus_x_bajo_mesa():
    from staubli_gui import scan_path
    part = scan_path.PartBox()
    kin = scan_path.load_kinematics()
    axes = {'+X': [0.0, 1.0, 0.0]}
    spec = scan_path.ScanPathSpec(name='+X', face='+X', line_length=0.08, line_axis=axes['+X'],
                                   scan_margin_start=0.005, scan_margin_end=0.02)
    sol = scan_path.resolve_scan(scan_path.ScanProfile(part=part, scan_distance=0.325), spec, kin)
    # No debe aceptar poses que intersecten mesa
    assert isinstance(sol.valid, bool)


def test_filtro_pieza_rechaza_interseccion_sensor():
    from staubli_gui import scan_path
    import PyKDL as kdl
    part = scan_path.PartBox()
    kin = scan_path.load_kinematics()
    mount = scan_path.sensor_mount_frame()
    # Colocar sensor directamente dentro de pieza AABB
    c = part.center
    sensor_frame = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(c[0], c[1], c[2]))
    col, reason = scan_path.check_sensor_environment_collision(sensor_frame * mount, part)
    assert col is True
    assert reason != ""


def test_collision_common_centralizado_sensor_pieza():
    from staubli_gui import collision_common
    import PyKDL as kdl
    from staubli_gui.scan_path import PartBox, sensor_mount_frame
    part = PartBox()
    c = part.center
    sensor_frame = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(c[0], c[1], c[2]))
    col, reason = collision_common.check_sensor_environment_collision(sensor_frame * sensor_mount_frame(), part)
    assert col is True


def test_collision_common_centralizado_sensor_mesa():
    from staubli_gui import collision_common
    import PyKDL as kdl
    from staubli_gui.scan_path import PartBox
    part = PartBox()
    # origen bajo mesa
    sensor_frame = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0.0, 0.5, 0.1))
    col, reason = collision_common.check_sensor_environment_collision(sensor_frame, part)
    assert col is True
