"""Planificación del barrido: de ScanSolution a waypoints articulares.

CAPA DE PLANIFICACIÓN (sin Qt, sin ROS, sin Kinematics)
=======================================================
Traduce la solución geométrica que produce `scan_path.resolve_scan()` en la
secuencia de waypoints que el `joint_trajectory_controller` va a ejecutar:

    scan_path.ScanSolution  ->  scan_execution  ->  waypoints (q, tiempo)

Decisiones de diseño (fase v1, validación del barrido real)
----------------------------------------------------------
1. **El `q` de cada punto se USA TAL CUAL**, no se recalcula. `resolve_scan()`
   ya encadena la IK (`q_prev`) y valida `solved` + `inside_limits`. Volver a
   resolver la IK aquí podría elegir otra rama articular o, peor, rehacer la
   orientación con `look_at_part_center()`, que apunta el **+Z de tool0** al
   centro de la pieza en lugar del **+X real de camera_link**; eso destruiría
   la orientación de barrido. Aquí no hay ninguna geometría duplicada: este
   módulo no sabe qué son las caras del cubo ni qué es el montaje de la cámara,
   solo consume `ScanPoint.q`.

2. **Un solo segmento.** El barrido es exclusivamente `P1 -> P2`
   (`SegmentoEscaneo`). No hay punto de retirada: la trayectoria termina en P2.

3. **Sin densificar (v1).** `SegmentoEscaneo` emite exactamente sus extremos.
   El JTC solo recibe `positions`, así que interpola **linealmente en espacio
   articular** entre P1 y P2: eso produce un arco, NO la recta cartesiana
   (desvío medido ~3.3 mm sobre 80 mm, y 0.005° de error angular). Densificar
   es una decisión posterior que entra por `SegmentoEscaneo.samples()` sin
   tocar nada más de la arquitectura.

4. **Sin velocidades (v1).** El goal lleva solo `positions` + `time_from_start`,
   de ahí que el JTC use interpolación lineal. Cuando se estudio suavizar el
   movimiento bastará con rellenar `JointTrajectoryPoint.velocities` para que
   el controlador eleve su spline a cúbico.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

from .scan_path import ScanSolution

# Tiempo provisional de la v1 (NO es un valor definitivo: no tiene respaldo
# físico todavía, es solo para observar el movimiento).
#   P1 -> P2: 80 mm en 1.0 s -> 80.1 mm/s, un 7.9% del límite articular.
DEFAULT_T_P1 = 0.0
DEFAULT_T_SCAN = 1.0

# El barrido completo son sus dos extremos; no hay roles de retirada.
SCAN_ROLES = ("P1", "P2")


@dataclass(frozen=True)
class Waypoint:
    """Un waypoint articular listo para el `JointTrajectory`."""

    name: str
    role: str
    q: List[float]
    time_from_start: float
    segment: str

    @property
    def joint_positions(self) -> List[float]:
        """q como floats planos, que es lo que espera el mensaje."""
        return [float(v) for v in self.q]


@dataclass(frozen=True)
class SegmentoEscaneo:
    """Tramo de ESCANEO: P1 -> P2 sobre la pieza, por encima de la mesa.

    En la v1 emite solo los dos extremos; es el contenedor donde entrará la
    densificación por interpolación cartesiana + IK.
    """

    start: Waypoint
    end: Waypoint

    @property
    def duration(self) -> float:
        return self.end.time_from_start - self.start.time_from_start

    def samples(self) -> List[Waypoint]:
        """Waypoints del tramo en orden temporal (extremos en la v1)."""
        return [self.start, self.end]

    @property
    def label(self) -> str:
        return "ESCANEO"


@dataclass(frozen=True)
class ScanExecutionPlan:
    """Plan completo del barrido: un único segmento de escaneo P1 -> P2."""

    scan: SegmentoEscaneo

    @property
    def waypoints(self) -> List[Waypoint]:
        """Secuencia total que se envía al JTC."""
        return [self.scan.start, self.scan.end]

    @property
    def total_duration(self) -> float:
        return self.scan.end.time_from_start

    def summary_lines(self) -> List[str]:
        """Líneas de log que dejan claro qué es barrido."""
        return [f"P1 -> P2: {self.scan.label} ({self.scan.duration:.2f} s)"]

    def segment_of(self, waypoint: Waypoint) -> str:
        return waypoint.segment


def _require_point(solution: ScanSolution, role: str):
    point = solution.point(role)
    if point is None:
        raise ValueError(f"la solución no contiene el punto {role}")
    return point


def _as_q(point) -> List[float]:
    """Copia defensiva del `q` de `scan_path` (no se recalcula la IK)."""
    return [float(v) for v in point.q]


def build_scan_execution(
    solution: ScanSolution,
    *,
    t_p1: float = DEFAULT_T_P1,
    t_scan: float = DEFAULT_T_SCAN,
    names: Optional[dict] = None,
) -> ScanExecutionPlan:
    """Convierte una `ScanSolution` en el plan de ejecución del barrido.

    `t_p1` es el tiempo absoluto de P1 (0.0) y `t_scan` la duración del tramo de
    escaneo. Los dos tiempos resultantes deben ser estrictamente crecientes.

    No se resuelve ninguna IK: se reutiliza `ScanPoint.q` tal cual, con la
    orientación de `scan_path` ya incorporada en esos valores.
    """
    if not solution.valid:
        raise ValueError(
            "la solución de barrido no es válida (IK sin solución o fuera de "
            "límites): no se puede planificar la ejecución"
        )

    p1 = _require_point(solution, "P1")
    p2 = _require_point(solution, "P2")

    t2 = float(t_p1) + float(t_scan)
    if not float(t_p1) < t2:
        raise ValueError(f"tiempos no crecientes: P1={t_p1}, P2={t2}")

    labels = dict(names or {})
    nm = lambda role, suffix: labels.get(role, f"scan_{suffix}_{role}")

    w_p1 = Waypoint(nm("P1", "ini"), "P1", _as_q(p1), float(t_p1), "ESCANEO")
    w_p2 = Waypoint(nm("P2", "fin"), "P2", _as_q(p2), t2, "ESCANEO")

    return ScanExecutionPlan(scan=SegmentoEscaneo(start=w_p1, end=w_p2))


def waypoints_to_dicts(plan: ScanExecutionPlan) -> List[dict]:
    """Forma plana para `TrajectoryBuilder.build_from_joint_waypoints()`."""
    return [
        {
            "name": wp.name,
            "role": wp.role,
            "q": wp.joint_positions,
            "time_from_start": wp.time_from_start,
            "segment": wp.segment,
        }
        for wp in plan.waypoints
    ]


def describe_plan(plan: ScanExecutionPlan) -> str:
    """Resumen multilínea para el log de la GUI."""
    lines = [
        f"barrido P1 -> P2 ({plan.scan.label}): "
        f"{len(plan.scan.samples())} muestras, {plan.scan.duration:.2f} s",
        f"waypoints enviados al JTC: {len(plan.waypoints)} "
        f"(sin velocidades -> interpolación lineal del JTC)",
    ]
    for wp in plan.waypoints:
        lines.append(
            f"  {wp.name:<18} rol={wp.role} t={wp.time_from_start:.2f}s "
            f"[{', '.join(f'{v:+.4f}' for v in wp.joint_positions)}]"
        )
    return "\n".join(lines)
