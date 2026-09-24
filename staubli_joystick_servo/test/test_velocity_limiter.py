"""Tests del control cartesiano por velocidad y su protección de singularidad.

Cubren los requisitos del rediseño de solve_qdot_dls() (DLS direccional SVD +
smoothstep):

1. HOME (q=0, Jacobiano de rango 3): X, RY y RZ deben producir movimiento;
   Y, Z y RX deben quedar a cero por ser direcciones físicamente inalcanzables.
2. Barrido HOME->P1: continuidad, sin saltos de salida no nula a "cero absoluto".
3. P1/P2/P3 (fuera de singularidad): sin regresión frente al DLS clásico.
4. Los límites de velocidad articular existentes siguen actuando.

Usa el Jacobiano REAL del TX2-60L (Tx260lKinematics) sobre la misma cadena que
el nodo de control, y los mismos parámetros de config/servo_params.yaml
(damping_lambda=0.05, singularity_threshold=0.02).
"""

import numpy as np
import pytest

from staubli_joystick_servo.velocity_limiter import (
    scale_to_joint_velocity_limits,
    solve_qdot_dls,
)
from staubli_trajectory.tx2_60l_ik import JOINT_VELOCITY_LIMITS_RAD, Tx260lKinematics

_LAMBDA = 0.05
_THRESHOLD = 0.02

_KIN = Tx260lKinematics()

_HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
_P1 = [0.0, -1.2, -1.2, 0.0, 1.0, 0.0]
_P2 = [0.5, -1.0, -1.0, 0.0, 0.8, 0.3]
_P3 = [-0.5, -1.2, -1.1, 0.2, 1.0, -0.2]
_POSES_FULL_RANK = {"P1": _P1, "P2": _P2, "P3": _P3}

_DOFS = {
    "X": np.array([1, 0, 0, 0, 0, 0], dtype=float),
    "Y": np.array([0, 1, 0, 0, 0, 0], dtype=float),
    "Z": np.array([0, 0, 1, 0, 0, 0], dtype=float),
    "RX": np.array([0, 0, 0, 1, 0, 0], dtype=float),
    "RY": np.array([0, 0, 0, 0, 1, 0], dtype=float),
    "RZ": np.array([0, 0, 0, 0, 0, 1], dtype=float),
}


def _qdot_at(q, dof):
    jac = _KIN.jacobian(q)
    return solve_qdot_dls(jac, _DOFS[dof], _LAMBDA, _THRESHOLD)


def _norm(q):
    return float(np.linalg.norm(q))


# ---------------------------------------------------------------------------
# 1) HOME: direcciones alcanzables vs inalcanzables
# ---------------------------------------------------------------------------


def test_home_reachable_direction_produces_motion():
    for dof in ("X", "RY", "RZ"):
        q_dot, _, _, _ = _qdot_at(_HOME, dof)
        assert _norm(q_dot) > 1.0e-3, f"Dirección alcanzable {dof} no produce movimiento en HOME"


def test_home_unreachable_directions_remain_zero():
    for dof in ("Y", "Z", "RX"):
        q_dot, _, _, _ = _qdot_at(_HOME, dof)
        assert _norm(q_dot) < 1.0e-9, f"Dirección {dof} es inalcanzable en HOME pero genera movimiento"


def test_home_jacobian_is_singular_rank_three():
    jac = _KIN.jacobian(_HOME)
    sigma = np.linalg.svd(jac, compute_uv=False)
    assert np.linalg.matrix_rank(jac) == 3
    assert float(np.min(sigma)) < 1.0e-9
    # La protección de singularidad debe estar activa en HOME.
    _, sigma_min, scale, _ = _qdot_at(_HOME, "X")
    assert sigma_min < 1.0e-9
    assert scale < 0.01


# ---------------------------------------------------------------------------
# 2) Barrido HOME->P1: continuidad sin saltos a "cero absoluto"
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("dof", ["X", "Y", "Z"])
def test_sweep_home_to_p1_is_continuous_and_never_jumps_to_zero(dof):
    q_dir = np.asarray(_P1, dtype=float)
    # Muestreo denso en la rampa de salida de la singularidad (HOME), donde la
    # ganancia crece más rápido, y más espaciado en la región bien condicionada.
    s_vals = np.concatenate([np.linspace(0.0, 0.2, 41), np.linspace(0.2, 1.0, 33)])
    eff = []
    for s in s_vals:
        q = list(s * q_dir)
        jac = _KIN.jacobian(q)
        raw, _, _, _ = solve_qdot_dls(jac, _DOFS[dof], _LAMBDA, _THRESHOLD)
        out = scale_to_joint_velocity_limits(raw, JOINT_VELOCITY_LIMITS_RAD)
        eff.append(_norm(out))
    eff = np.asarray(eff)

    # En HOME: X es alcanzable, Y/Z no lo son físicamente (y deben arrancar en 0).
    if dof == "X":
        assert eff[0] > 1.0e-3
    else:
        assert eff[0] < 1.0e-6

    # La salida nunca es nula una vez ha empezado a serlo (sin colapso a cero
    # absoluto cruzando la singularidad ni en ningún punto intermedio del barrido).
    if dof == "X":
        assert np.all(eff > 1.0e-3), "X se anula en algún punto del barrido"
    started = False
    for e in eff:
        if e > 1.0e-3:
            started = True
        if started:
            assert e > 1.0e-3, f"{dof} salta de no-nulo a cero absoluto en el barrido"

    # Variación entre muestras consecutivas acotada (continuidad, sin escalones),
    # incluida la rama de transición HOME -> región bien condicionada.
    diffs = np.abs(np.diff(eff))
    assert float(np.max(diffs)) < 0.6, f"{dof}: salto abrupto entre muestras del barrido"

    # Las velocidades nunca explotan (límites articulares respetados post-scalado).
    max_allowed = float(np.sqrt(5 * 5.236**2 + 8.7266**2))  # peor caso con joint_6
    assert float(np.max(eff)) <= max_allowed + 1.0e-6


@pytest.mark.parametrize("dof", ["Y", "Z"])
def test_sweep_unreachable_dofs_ramp_gradually_from_home(dof):
    # La dirección pasa de "cero en HOME" a "capaz" de forma progresiva: los
    # primeros valores no-nulos deben ser pequeños y crecer sin saltos (sin el
    # pico de amplificación 1/(2·lambda) del DLS clásico en la salida de la
    # singularidad).
    q_dir = np.asarray(_P1, dtype=float)
    s_vals = np.linspace(0.0, 0.12, 25)
    eff = []
    for s in s_vals:
        q = list(s * q_dir)
        jac = _KIN.jacobian(q)
        raw, _, _, _ = solve_qdot_dls(jac, _DOFS[dof], _LAMBDA, _THRESHOLD)
        eff.append(_norm(scale_to_joint_velocity_limits(raw, JOINT_VELOCITY_LIMITS_RAD)))
    eff = np.asarray(eff)

    # Arranque en cero y crecimiento continuo en los primeros pasos.
    assert eff[0] < 1.0e-6
    assert eff[1] > 1.0e-6
    assert float(np.max(np.abs(np.diff(eff)))) < 0.6, f"{dof}: arranque brusco desde HOME"


def test_sweep_active_near_singular_region():
    # Con sigma_min por debajo del umbral la autoridad debe degradarse de forma
    # SUAVE (continuidad), pero seguir habiendo respuesta en X.
    q_dir = np.asarray(_P1, dtype=float)
    n_deg = n_sing = 0
    ok = True
    for s in np.linspace(0.0, 0.15, 31):
        q = list(s * q_dir)
        jac = _KIN.jacobian(q)
        sigma_min = float(np.min(np.linalg.svd(jac, compute_uv=False)))
        if sigma_min < _THRESHOLD:
            n_sing += 1
            raw, _, _, _ = solve_qdot_dls(jac, _DOFS["X"], _LAMBDA, _THRESHOLD)
            if _norm(raw) < 1.0e-3:
                ok = False
        else:
            n_deg += 1
    assert n_sing > 0, "El barrido no cruza la región singular"
    assert n_deg > 0, "El barrido no cruza la región bien condicionada"
    assert ok, "Se anula el movimiento de X dentro de la región singular (transición dura)"


# ---------------------------------------------------------------------------
# 3) P1/P2/P3: sin regresión frente al DLS clásico (sigma_min >= threshold)
# ---------------------------------------------------------------------------


def _legacy_qdot_dls(jac, twist, lam, th):
    """Referencia del comportamiento anterior (DLS + guard global de singularidad)."""
    jac = np.asarray(jac, dtype=float)
    twist = np.asarray(twist, dtype=float).reshape(6)
    sigma = np.linalg.svd(jac, compute_uv=False)
    sigma_min = float(np.min(sigma))
    scale = 1.0
    lam_eff = float(lam)
    if sigma_min < th:
        scale = max(0.0, sigma_min / th)
        lam_eff = max(lam_eff, lam * th / max(sigma_min, 1.0e-9))
    a_mat = jac @ jac.T + lam_eff**2 * np.eye(6)
    return jac.T @ np.linalg.solve(a_mat, twist * scale)


@pytest.mark.parametrize("pose_name", list(_POSES_FULL_RANK))
@pytest.mark.parametrize("dof", list(_DOFS))
def test_full_rank_poses_match_legacy_behavior(pose_name, dof):
    q = _POSES_FULL_RANK[pose_name]
    jac = _KIN.jacobian(q)
    sigma_min = float(np.min(np.linalg.svd(jac, compute_uv=False)))
    assert sigma_min >= _THRESHOLD, f"{pose_name} debería estar fuera de la singularidad"

    new, new_smin, new_scale, new_lambda = solve_qdot_dls(
        jac, _DOFS[dof], _LAMBDA, _THRESHOLD
    )
    legacy = _legacy_qdot_dls(jac, _DOFS[dof], _LAMBDA, _THRESHOLD)

    assert np.allclose(new, legacy, atol=1.0e-8)
    assert new_scale == pytest.approx(1.0)
    assert new_lambda == pytest.approx(_LAMBDA)


# ---------------------------------------------------------------------------
# 4) Los límites de velocidad articular existentes siguen actuando
# ---------------------------------------------------------------------------


def test_joint_velocity_limits_still_clamp_overspeed_commands():
    # Con el límite de ganancia por dirección, un pulso único nunca excede los
    # límites (bien condicionado); se comprueba que, cuando el solver SÍ pide
    # velocidades excesivas (comando combinado máximo escalado en P3, en el que
    # el límite cartesiano se aplica antes del DLS en el pipeline real), el
    # limitador articular existente sigue recortando hasta cada límite.
    q = _P3
    jac = _KIN.jacobian(q)
    # Twist máximo que admiten clamp_vector (max_linear_speed=0.15, max_angular_speed=0.5),
    # combinado en todas las direcciones y escalado para excitar varios modos.
    twist_cap = 6.0 * np.array([0.15, 0.15, 0.15, 0.5, 0.5, 0.5], dtype=float)
    raw, _, _, _ = solve_qdot_dls(jac, twist_cap, _LAMBDA, _THRESHOLD)

    ratio_raw = float(np.max(np.abs(raw) / np.asarray(JOINT_VELOCITY_LIMITS_RAD)))
    assert ratio_raw > 1.0, "El caso de test no dispara el limitador (test vacío)"

    out = scale_to_joint_velocity_limits(raw, JOINT_VELOCITY_LIMITS_RAD)
    ratio_out = np.abs(out) / np.asarray(JOINT_VELOCITY_LIMITS_RAD)
    assert float(np.max(ratio_out)) <= 1.0 + 1.0e-9
    # Al menos una articulación debe estar clavada exactamente en su límite.
    assert float(np.max(ratio_out)) == pytest.approx(1.0, abs=1.0e-6)


def test_joint_velocity_limits_respected_across_sweep():
    q_dir = np.asarray(_P1, dtype=float)
    for s in np.linspace(0.0, 1.0, 17):
        q = list(s * q_dir)
        jac = _KIN.jacobian(q)
        for dof in _DOFS:
            raw, _, _, _ = solve_qdot_dls(jac, _DOFS[dof], _LAMBDA, _THRESHOLD)
            out = scale_to_joint_velocity_limits(raw, JOINT_VELOCITY_LIMITS_RAD)
            assert float(np.max(np.abs(out) / np.asarray(JOINT_VELOCITY_LIMITS_RAD))) <= 1.0 + 1.0e-9