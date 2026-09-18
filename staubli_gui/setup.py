import os
from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "staubli_gui"

setup(
    name=PACKAGE_NAME,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rocio",
    maintainer_email="rocio@example.com",
    description=(
        "Interfaz gráfica PySide6 de control del Staubli TX2-60L (fase 1: solo visual)."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gui_node = staubli_gui.gui:main",
        ],
    },
)
