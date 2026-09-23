"""Punto de entrada de build/install para el paquete ament_python.

Convenciones ROS 2 estándar:
    - packages            ~> descubrimiento automático (find_packages)
    - data_files          ~> instala a share/<pkg>: package.xml, config/*.yaml
                            (-> share/<pkg>/config) y launch/*.launch.py
                            (-> share/<pkg>/launch), además del marcador
                            resource/<pkg> para el resource_index.
    - entry_points        ~> genera el ejecutable 'servo_node', de modo que
                            se lanza con `ros2 run staubli_joystick_servo
                            servo_node` con los parámetros en config YAML.
"""

import os
from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "staubli_joystick_servo"

setup(
    name=PACKAGE_NAME,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        (os.path.join("share", PACKAGE_NAME, "config"), glob("config/*.yaml")),
        (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rocio",
    maintainer_email="rocio@example.com",
    description=(
        "Control cartesiano por velocidad del TX2-60L (base_link): "
        "/cmd_vel -> Jacobiano PyKDL (DLS) -> /forward_velocity_controller/commands."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "servo_node = staubli_joystick_servo.servo_node:main",
        ],
    },
)