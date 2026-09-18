"""Launch file del nodo de trayectoria cartesiana del Staubli TX2-60L.

Lanza el nodo 'staubli_trajectory_node' con los argumentos propios:

  --config   Ruta a config/points.yaml  (por defecto la instalada en el
             share del paquete, share/<pkg>/config/points.yaml; puede
             sobreescribirse con `ros2 launch ... config:=/ruta/otra.yaml`).
  --send     Si el argumento de launch 'send' vale 'true', además de calcular
             e imprimir, ENVÍA la trayectoria al controller
             (/joint_trajectory_controller/follow_joint_trajectory).
             AVISO: solo hay que activarlo con Gazebo + controllers activos.

Uso:
    ros2 launch staubli_trajectory trajectory.launch.py                 # cálculo
    ros2 launch staubli_trajectory trajectory.launch.py send:=true      # envía

El selector entre "sin envío" (por defecto) y "con envío" se implementa con
dos declaraciones del mismo Node condicionadas (IfCondition/UnlessCondition)
sobre el valor del argumento 'send'.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Ruta por defecto de la configuración (la instalada con el paquete).
    default_config = os.path.join(
        get_package_share_directory("staubli_trajectory"),
        "config",
        "points.yaml",
    )

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description=(
            "Ruta a config/points.yaml (posiciones cartesianas de tool0, "
            "centro de la pieza, tiempos y tolerancias)."
        ),
    )

    send_arg = DeclareLaunchArgument(
        "send",
        default_value="false",
        description=(
            "true => además de calcular, ENVÍA la trayectoria al "
            "joint_trajectory_controller (requiere Gazebo + controllers)."
        ),
    )

    # Modo seguro (por defecto): SOLO calcula e imprime el plan.
    node_calculate = Node(
        package="staubli_trajectory",
        executable="trajectory_node",
        name="staubli_trajectory_node",
        output="screen",
        arguments=["--config", LaunchConfiguration("config")],
        condition=UnlessCondition(LaunchConfiguration("send")),
    )

    # Modo explícito (send:=true): además de calcular, envía al controller.
    node_send = Node(
        package="staubli_trajectory",
        executable="trajectory_node",
        name="staubli_trajectory_node",
        output="screen",
        arguments=[
            "--config",
            LaunchConfiguration("config"),
            "--send",
        ],
        condition=IfCondition(LaunchConfiguration("send")),
    )

    return LaunchDescription([config_arg, send_arg, node_calculate, node_send])