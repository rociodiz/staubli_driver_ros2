"""Utilidades numéricas del control cartesiano por velocidad (TX2-60L).

Módulo SIN dependencias de ROS (solo NumPy), pensado para ser testeable y
reutilizable:

- apply_deadzone():  deadzone por eje (lineal/angular).
- clamp_vector():    límite cartesiano global manteniendo la dirección.
- solve_qdot_dls():  q_dot = J^T (J J^T + lambda^2 I)^-1 Twist, con
                     protección de singularidades por SVD (sigma_min).
- scale_to_joint_velocity_limits(): escala proporcional si algún q_dot
                     supera su límite articular.
- JointVelocityLimiter: rampas de aceleración/desaceleración y detención
                     inmediata (force_zero) para STOP/watchdog.
"""

from __future__ import annotations

from typing import List, Sequence

import numpy as np


def apply_deadzone(
    linear: Sequence[float],
    angular: Sequence[float],
    deadzone_linear: float,
    deadzone_angular: float,
) -> tuple[List[float], List[float]]:
    """Aplica deadzone por eje del Twist: |v| < umbral -> 0.0."""
    lin = [0.0 if abs(v) < deadzone_linear else float(v) for v in linear]
    ang = [0.0 if abs(v) < deadzone_angular else float(v) for v in angular]
    return lin, ang


def clamp_vector(vector: Sequence[float], limit: float) -> List[float]:
    """Escala el vector proporcionalmente si su norma supera `limit`.

    Mantiene la dirección del movimiento (escalado uniforme).
    """
    vec = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(vec))
    if limit > 0.0 and norm > limit:
        scale = limit / norm
        return [float(v * scale) for v in vec]
    return [float(v) for v in vec]


def solve_qdot_dls(
    jacobian: np.ndarray,
    twist: Sequence[float],
    damping_lambda: float = 0.05,
    singularity_threshold: float = 0.02,
) -> tuple[np.ndarray, float, float, float]:
    """Resuelve q_dot = J^T (J J^T + lambda^2 I)^-1 Twist (DLS).

    Protección de singularidad:
    - Si sigma_min < singularity_threshold, se reduce el Twist por un factor
      scale = sigma_min / threshold (0..1) y se aumenta el amortiguamiento.
    - Nunca se deja explotar q_dot.

    Devuelve (q_dot, sigma_min, scale, lambda_eff).
    """
    jac = np.asarray(jacobian, dtype=float)
    twist = np.asarray(twist, dtype=float).reshape(6)

    sigma = np.linalg.svd(jac, compute_uv=False)
    sigma_min = float(np.min(sigma))

    scale = 1.0
    lambda_eff = float(damping_lambda)
    if sigma_min < singularity_threshold:
        scale = max(0.0, sigma_min / singularity_threshold)
        lambda_eff = max(
            lambda_eff,
            damping_lambda * singularity_threshold / max(sigma_min, 1.0e-9),
        )

    twist_scaled = twist * scale
    a_mat = jac @ jac.T + lambda_eff**2 * np.eye(6)
    q_dot = jac.T @ np.linalg.solve(a_mat, twist_scaled)
    return q_dot, sigma_min, scale, lambda_eff


def scale_to_joint_velocity_limits(
    q_dot: Sequence[float], limits: Sequence[float]
) -> np.ndarray:
    """Escala proporcionalmente q_dot si alguna articulación supera su límite.

    Mantiene la dirección del vector (escalado uniforme del mínimo ratio).
    """
    q = np.asarray(q_dot, dtype=float)
    ratio = 1.0
    for v, lim in zip(q, limits):
        if lim > 0.0 and abs(v) > lim:
            ratio = min(ratio, lim / abs(v))
    if ratio < 1.0:
        q = q * ratio
    return q


class JointVelocityLimiter:
    """Aplica rampas de aceleración y mantiene la última salida.

    - update(target): aproxima la salida al objetivo en pasos de a lo sumo
      acceleration_limit * dt por articulación (frenado progresivo incluido).
    - force_zero(): detención INMEDIATA (STOP / watchdog), sin rampa.
    """

    def __init__(
        self,
        joint_velocity_limits: Sequence[float],
        acceleration_limit: float,
        control_rate: float,
    ) -> None:
        self._limits = [float(v) for v in joint_velocity_limits]
        self._acc_limit = float(acceleration_limit)
        self._dt = 1.0 / float(control_rate)
        self._qdot = np.zeros(len(self._limits))

    @property
    def last_output(self) -> np.ndarray:
        return np.array(self._qdot)

    def reset(self) -> np.ndarray:
        self._qdot = np.zeros(len(self._limits))
        return np.array(self._qdot)

    def force_zero(self) -> np.ndarray:
        return self.reset()

    def update(self, target_qdot: Sequence[float], dt: float | None = None) -> np.ndarray:
        dt = self._dt if dt is None else float(dt)
        target = np.asarray(target_qdot, dtype=float)
        step = self._acc_limit * dt
        out = np.array(self._qdot)
        for i in range(len(out)):
            diff = float(target[i]) - out[i]
            if abs(diff) <= step:
                out[i] = target[i]
            else:
                out[i] += step * (1.0 if diff > 0.0 else -1.0)
        self._qdot = out
        return np.array(out)