"""Punto de entrada de build/install para el paquete ament_python.

Convenciones ROS 2 estándar:
    - packages            ~> descubrimiento automático (find_packages)
    - data_files          ~> instala a share/<pkg>: package.xml, config/*.yaml
                            (-> share/<pkg>/config) y launch/*.launch.py
                            (-> share/<pkg>/launch), además del marcador
                            resource/<pkg> para el resource_index.
    - entry_points        ~> genera el ejecutable 'trajectory_node', de modo
                            que se lanza con `ros2 run staubli_trajectory
                            trajectory_node` igual que antes (sin scripts/).
"""

import os
from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "staubli_trajectory"

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
        "Trayectorias cartesianas configurables del TX2-60L sin MoveIt: "
        "points.yaml -> IK PyKDL (tool0/base_link) -> FollowJointTrajectory."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "trajectory_node = staubli_trajectory.trajectory_node:main",
        ],
    },
)