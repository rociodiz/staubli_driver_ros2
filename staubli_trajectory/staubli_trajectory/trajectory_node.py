"""Nodo de trayectoria cartesiana para el Staubli TX2-60L (sin MoveIt).

Flujo:
    config/points.yaml  ->  PointsConfig  ->  TrajectoryBuilder (IK PyKDL)
    ->  JointTrajectory  ->  [--send] JointTrajectoryClient (FollowJointTrajectory)

Por defecto el nodo SOLO CALCULA e IMPRIME en terminal, por cada punto:
    - posición cartesiana de tool0 (base_link),
    - quaternion (x, y, z, w) de la orientación usada,
    - q1..q6 resultantes de la IK,
    - si la solución está dentro de los límites articulares.
NO ejecuta ningún movimiento al arrancar.

Para enviar la trayectoria al controller hay que pedirlo explícitamente con
`--send` (y tener Gazebo + controllers activos):

    ros2 run staubli_trajectory trajectory_node --send
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import List, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from .joint_trajectory_client import JointTrajectoryClient
from .points_config import PointsConfig
from .trajectory_builder import TrajectoryBuilder, TrajectoryPlan, WaypointRecord


def _default_config_path() -> str:
    """Ruta de config/points.yaml instalada con el paquete."""
    pkg_share = get_package_share_directory("staubli_trajectory")
    return os.path.join(pkg_share, "config", "points.yaml")


# ---------------------------------------------------------------------------
def _print_plan(plan: TrajectoryPlan, config: PointsConfig) -> None:
    """Vuelca en terminal el diagnóstico completo de cada punto del plan."""
    banner = "=" * 78
    print(banner)
    print(f"PLAN DE TRAYECTORIA CARTESIANA  |  frame={config.planning_frame}"
          f"  |  mode={config.orientation_mode}")
    print(banner)
    for i, (point, record) in enumerate(zip(config.points, plan.records), start=1):
        _print_point(i, point, record)
        print("-" * 78)
    resumen(plan)


def _print_point(i: int, point: dict, record: WaypointRecord) -> None:
    print(f"Punto {i} ({record.name})  [{record.info}]")
    print(f"  posición cartesiana : "
          f"({record.cartesian_position[0]:+.4f}, "
          f"{record.cartesian_position[1]:+.4f}, "
          f"{record.cartesian_position[2]:+.4f})  m")
    q = record.quaternion_xyzw
    print(f"  quaternion (xyzw)   : "
          f"({q[0]:+.4f}, {q[1]:+.4f}, {q[2]:+.4f}, {q[3]:+.4f})")
    if not record.solved:
        print("  q1..q6 (rad)        : ---  (IK SIN SOLUCIÓN)")
        return
    qs = ", ".join(f"{v:+.4f}" for v in record.q)
    print(f"  q1..q6 (rad)        : [{qs}]")
    if record.inside_limits:
        print("  límites articulares : DENTRO  [ok]")
    else:
        fuera = [f"joint_{k}" for k, ok in enumerate(record.joint_ok, start=1) if not ok]
        print(f"  límites articulares : FUERA de límites en {', '.join(fuera)}")


def resumen(plan: TrajectoryPlan) -> None:
    print(f"\nResumen: {len(plan.records)} puntos | "
          f"IK resuelta: {'SÍ' if plan.all_solved else 'NO'} | "
          f"dentro de límites: {'SÍ' if plan.all_inside_limits else 'NO'}")
    if not plan.all_solved:
        print("> No se generará JointTrajectory hasta resolver correctamente "
              "las IK de todos los puntos.")
    elif not plan.all_inside_limits:
        print("> La trayectoria es válida pero hay puntos fuera de límites: "
              "revisa coordenadas en config/points.yaml.")


# ---------------------------------------------------------------------------
class TrajectoryNode(Node):
    """Orquesta: configuración -> cálculo -> [envío opcional]."""

    def __init__(self, config_path: str, send: bool = False) -> None:
        super().__init__("staubli_trajectory_node")
        self._config_path = config_path
        self._send = send

    # ------------------------------------------------------------------
    def run(self) -> bool:
        try:
            config = PointsConfig(self._config_path)
        except Exception as exc:
            self.get_logger().error(f"Error de configuración: {exc}")
            return False

        self.get_logger().info(
            f"Config: '{config.config_path}' | frame={config.planning_frame} | "
            f"{len(config.points)} puntos | orientation_mode={config.orientation_mode}"
        )

        try:
            builder = TrajectoryBuilder(config)
            plan = builder.build()
        except Exception as exc:  # noqa: BLE001 - reporte claro al usuario
            self.get_logger().error(f"Error construyendo la trayectoria: {exc}")
            return False

        # Informe por terminal (SIEMPRE, sea o no envío).
        _print_plan(plan, config)

        if not self._send:
            self.get_logger().info(
                "Modo simulación (sin envío). Usa '--send' para mover el robot."
            )
            return plan.all_solved

        if plan.trajectory is None:
            self.get_logger().error(
                "No se puede enviar: la IK no produjo solución para todos los puntos."
            )
            return False
        if not plan.all_inside_limits:
            self.get_logger().error(
                "No se envía: hay puntos fuera de límites articulares."
            )
            return False

        client = JointTrajectoryClient(self)
        if not client.wait_for_server():
            return False
        return client.send(plan.trajectory, config)


# ---------------------------------------------------------------------------
def main(args: Optional[List[str]] = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default=None,
        help="Ruta alternativa a config/points.yaml.",
    )
    parser.add_argument(
        "--send",
        action="store_true",
        help="Además de calcular/imprimir, ENVÍA la trayectoria al controller.",
    )
    cli_args, desconocidos = parser.parse_known_args(args)

    # rclpy solo consume sus propios argumentos (los '--ros-args ...').
    rclpy.init(args=desconocidos or None)

    config_path = cli_args.config or _default_config_path()
    node = TrajectoryNode(config_path, send=cli_args.send)
    try:
        ok = node.run()
    except KeyboardInterrupt:
        ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()