"""Fixtures compartidos para los tests headless de staubli_joystick_servo.

Permite ejecutar `pytest test/` sin depender de que exista un entorno ROS ya
fuenteado:
- Añade la raíz del paquete al sys.path (import del propio paquete).
- Añade la ruta del paquete hermano 'staubli_trajectory' desde las fuentes
  (fallback cuando no hay un instalación ROS disponible), para que los tests
  puedan construir el Jacobiano real del TX2-60L.
"""

import sys
from pathlib import Path

_PKG_ROOT = Path(__file__).resolve().parents[1]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

_REPO_ROOT = _PKG_ROOT.parent
_TRAJ_SRC = _REPO_ROOT / "staubli_trajectory"
if _TRAJ_SRC.is_dir() and str(_TRAJ_SRC) not in sys.path:
    sys.path.insert(0, str(_TRAJ_SRC))