"""Fixtures compartidos para los tests headless de staubli_gui.

- Fuerza el backend Qt offscreen (sin pantalla).
- Aísla el almacén de puntos en un directorio temporal vía
  STAUBLI_GUI_CONFIG_DIR (nunca toca ~/.config del usuario).
"""

import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

_SRC_PKG_ROOT = Path(__file__).resolve().parents[1]
if str(_SRC_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_SRC_PKG_ROOT))

# Tx260lKinematics (paquete hermano staubli_trajectory). Basta con añadir la
# raíz del paquete: su lookup de config sube por el árbol de fuentes repo.
_TRAJ_PKG = _SRC_PKG_ROOT.parent / "staubli_trajectory"
if str(_TRAJ_PKG) not in sys.path:
    sys.path.insert(0, str(_TRAJ_PKG))

import pytest


@pytest.fixture(scope="session")
def qapp():
    """Instancia única de QApplication (imprescindible para widgets Qt)."""
    from PySide6.QtWidgets import QApplication

    app = QApplication.instance() or QApplication([])
    yield app


@pytest.fixture
def config_dir(tmp_path, monkeypatch):
    """Directorios de configuración aislados por test."""
    monkeypatch.setenv("STAUBLI_GUI_CONFIG_DIR", str(tmp_path))
    return tmp_path