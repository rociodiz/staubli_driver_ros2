"""Tests headless de la lógica de controladores (controller_manager.py).

No hay ROS ni robot: se contrastan las funciones puras derive_mode() y
switch_plan() que garantizan el modo EXCLUYENTE (nunca JTC y FVC activos a
la vez) y la presencia del joint_state_broadcaster en todos los planes.
"""

from staubli_gui.controller_manager import (
    ACTIVE_STATE,
    FORWARD_VELOCITY_CONTROLLER,
    INACTIVE_STATE,
    JOINT_STATE_BROADCASTER,
    JOINT_TRAJECTORY_CONTROLLER,
    derive_mode,
    switch_plan,
)

_JTC = JOINT_TRAJECTORY_CONTROLLER
_FVC = FORWARD_VELOCITY_CONTROLLER
_JSB = JOINT_STATE_BROADCASTER


def _states(extra=None):
    base = {_JTC: INACTIVE_STATE, _FVC: INACTIVE_STATE, _JSB: ACTIVE_STATE}
    if extra:
        base.update(extra)
    return base


# ── derive_mode ─────────────────────────────────────────────────────────

def test_derive_mode_trayectoria():
    st = _states({_JTC: ACTIVE_STATE, _FVC: INACTIVE_STATE})
    assert derive_mode(st) == "trayectoria"


def test_derive_mode_manual():
    st = _states({_JTC: INACTIVE_STATE, _FVC: ACTIVE_STATE})
    assert derive_mode(st) == "manual"


def test_derive_mode_seguro():
    st = _states({_JTC: INACTIVE_STATE, _FVC: INACTIVE_STATE})
    assert derive_mode(st) == "seguro"


def test_derive_mode_conflicto():
    st = _states({_JTC: ACTIVE_STATE, _FVC: ACTIVE_STATE})
    assert derive_mode(st) == "conflicto"


def test_derive_mode_desconocido():
    assert derive_mode({}) == "desconocido"


# ── switch_plan (exclusividad y JSB siempre activo) ─────────────────────

def test_plan_nunca_activa_ambos_controladores():
    for modo in ("trayectoria", "manual", "seguro"):
        plan = switch_plan(modo)
        activos = set(plan["activate"])
        assert not (_JTC in activos and _FVC in activos)


def test_plan_trayectoria_activa_jtc_y_desactiva_fvc():
    plan = switch_plan("trayectoria")
    assert _JTC in plan["activate"]
    assert _FVC in plan["deactivate"]
    assert _JSB in plan["activate"]


def test_plan_manual_activa_fvc_y_desactiva_jtc():
    plan = switch_plan("manual")
    assert _FVC in plan["activate"]
    assert _JTC in plan["deactivate"]
    assert _JSB in plan["activate"]


def test_plan_seguro_desactiva_ambos():
    plan = switch_plan("seguro")
    assert _JTC in plan["deactivate"]
    assert _FVC in plan["deactivate"]
    assert _JSB in plan["activate"]
    assert _JTC not in plan["activate"]
    assert _FVC not in plan["activate"]


def test_plan_modo_desconocido_es_vacio():
    plan = switch_plan("desconocido")
    assert plan == {"activate": (), "deactivate": ()}