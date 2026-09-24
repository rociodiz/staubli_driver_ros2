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


def _smoothstep(x: float) -> float:
    """Rampa suave C^1 en [0, 1] usada como factor de alcance por dirección."""
    x = float(np.clip(x, 0.0, 1.0))
    return x * x * (3.0 - 2.0 * x)


# Valor de sigma (escala del Jacobiano, rad/s) a partir del cual se restaura la
# ganancia DLS 1/sigma sin límite. Por debajo, la ganancia se acota a la rampa
# lineal sigma/(sigma_rejoin^2+lambda^2), que nace en 0 de forma continua y sin
# picos (elimina el amplificador 1/(2·lambda) del DLS clásico en la transición a
# la singularidad). Se elige claramente por DEBAJO del sigma_min de las poses de
# trabajo habituales (P1/P2/P3 ~ 0.17) para que éstas conserven EXACTAMENTE el
# comportamiento del DLS clásico.
_SIGMA_RECOVERY_FACTOR = 3.0


def solve_qdot_dls(
    jacobian: np.ndarray,
    twist: Sequence[float],
    damping_lambda: float = 0.05,
    singularity_threshold: float = 0.02,
) -> tuple[np.ndarray, float, float, float]:
    """Resuelve q_dot = J^T (J J^T + lambda^2 I)^-1 Twist (DLS direccional).

    Protección de singularidad POR DIRECCIÓN (SVD): se usa la descomposición
    J = U Σ V^T y la salida se construye modo a modo en el espacio del Twist:

        q_dot = Σ_i  gain(σ_i) · (u_iᵀ · Twist) · v_i
        gain(σ) = σ / (max(σ, σ_rejoin)² + lambda²)
        σ_rejoin = _SIGMA_RECOVERY_FACTOR · max(damping_lambda, singularity_threshold)

    Propiedades:
    - Fuera de la banda singular (σ_i >= σ_rejoin) gain(σ) = σ/(σ²+lambda²),
      idéntico al DLS clásico: en P1/P2/P3 (σ_min ~ 0.17 > 0.15) no hay
      regresión alguna.
    - Cada dirección se atenúa SOLO según su propio σ: los modos bien
      condicionados conservan la ganancia 1/σ y no se escala el Twist completo.
    - Por debajo de σ_rejoin la ganancia es una rampa lineal en σ que nace en 0
      (direcciones sin generador físico quedan exactamente a 0, sin falsear el
      movimiento) y cuyo máximo está acotado (≈ 1/σ_rejoin): sin picos q_dot
      enormes al cruzar la singularidad.
    - La transición es continua y monótona: no hay salto de "se mueve" a
      "cero absoluto".
    - No se introduce epsilon para forzar σ_min ni se infla lambda globalmente:
      el amortiguamiento efectivo es siempre el mismo damping_lambda.

    Devuelve (q_dot, sigma_min, scale, lambda_eff), donde scale =
    smoothstep(sigma_min / singularity_threshold) resume la autoridad retenida
    (1.0 fuera de la singularidad) y lambda_eff es el amortiguamiento constante
    usado.
    """
    jac = np.asarray(jacobian, dtype=float)
    twist = np.asarray(twist, dtype=float).reshape(6)

    sigma_rejoin = _SIGMA_RECOVERY_FACTOR * max(damping_lambda, singularity_threshold)

    u, sigma, vh = np.linalg.svd(jac, full_matrices=False)
    sigma_min = float(np.min(sigma))
    scale = _smoothstep(sigma_min / singularity_threshold)

    q_dot = np.zeros(6)
    for i, sig in enumerate(sigma):
        if sig > 0.0:
            gain = sig / (max(sig, sigma_rejoin) ** 2 + damping_lambda**2)
            q_dot += gain * float(u[:, i] @ twist) * vh[i]
    return q_dot, sigma_min, float(scale), float(damping_lambda)


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