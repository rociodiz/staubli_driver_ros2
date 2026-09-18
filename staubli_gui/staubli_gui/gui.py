"""Ventana principal de la GUI del Staubli TX2-60L.

Integración ROS 2 (fase 1, solo lectura): el nodo se suscribe a
/joint_states (sensor_msgs/msg/JointState) con QoS RELIABLE +
TRANSIENT_LOCAL + KEEP_LAST(1) y muestra en tiempo real las posiciones
joint_1..joint_6 -> J1..J6 (radianes, 4 decimales). El bucle ROS se
integra con el ciclo de eventos de Qt mediante un QTimer que ejecuta
rclpy.spin_once(...) periódicamente. Sin conexión con actions/services:
ENABLE, DISABLE, HOME, P1-P3, EJECUTAR TRAYECTORIA y STOP siguen siendo
placeholders.
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


# ─── Colores ───────────────────────────────────────────────────────────
_ROJO       = "#c0392b"
_ROJO_CLARO = "#e74c3c"
_VERDE      = "#27ae60"
_AZUL       = "#2980b9"
_GRIS_OSC   = "#2c3e50"
_GRIS_MED   = "#34495e"
_GRIS_CLR   = "#ecf0f1"
_NEGRO      = "#1a1a2e"
_BLANCO     = "#ffffff"
_BLANCO_HUESO = "#f5f6fa"
_AMARILLO_BRILLANTE = "#ffd75e"
_AMARILLO   = "#f1c40f"

# Fuentes explícitas para evitar fallos de renderizado/fallback del sistema.
_FAMILIA  = '"DejaVu Sans", "Noto Sans", "Liberation Sans", sans-serif'
_FAM_MONO = '"DejaVu Sans Mono", "Liberation Mono", "Noto Sans Mono", monospace'


# ─── ROS 2 (fase 1: lectura de J1-J6) ───────────────────────────────────
# Map Nombres ROS ("joint_1".."joint_6") -> etiquetas de la GUI ("J1".."J6").
_ROS_JOINT_MAP = {f"joint_{i}": f"J{i}" for i in range(1, 7)}

# QoS compatible con joint_state_broadcaster (ros2_control): RELIABLE,
# TRANSIENT_LOCAL (recibe el estado más reciente aunque llegue tarde) y
# KEEP_LAST con depth 1.
_JOINT_STATES_TOPIC = "/joint_states"
_JOINT_STATES_MSG = JointState
_JOINT_STATES_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# Frecuencia del QTimer que integra el bucle ROS con Qt (spin_once).
_ROS_SPIN_MS = 33  # ~30 Hz (no bloquea la GUI)


def _stylesheet() -> str:
    """Estilos de contenedores (ventana, group boxes, títulos).

    El texto/color de BOTONES y ETIQUETAS se fija por-widget (inline), con
    prioridad máxima, para que el estilo nativo del escritorio no lo invalide.
    """
    return f"""
    QMainWindow {{
        background-color: {_NEGRO};
    }}
    QGroupBox {{
        background-color: {_GRIS_OSC};
        border: 1px solid {_GRIS_MED};
        border-radius: 6px;
        margin-top: 10px;
        padding: 8px 10px 6px 10px;
        color: {_GRIS_CLR};
        font-family: {_FAMILIA};
        font-size: 13px;
        font-weight: bold;
    }}
    QGroupBox::title {{
        subcontrol-origin: margin;
        left: 12px;
        padding: 0 6px;
    }}
    """


# ─── Estilos por-widget (inline; máxima prioridad) ─────────────────────
def _btn_style(
    bg: str,
    fg: str = _BLANCO_HUESO,
    font_size: int = 13,
    min_w: int = 0,
    min_h: int = 36,
    pad: str = "8px 18px",
    extra: str = "",
) -> str:
    style = f"""
    QPushButton {{
        background-color: {bg};
        color: {fg};
        border: none;
        border-radius: 5px;
        padding: {pad};
        font-family: {_FAMILIA};
        font-size: {font_size}px;
        font-weight: bold;
        min-height: {min_h}px;
    }}
    """
    if min_w:
        style += f"QPushButton {{ min-width: {min_w}px; }}\n"
    return style + extra


def _label_style(fg: str, font_family: str, font_size: int, extra: str = "") -> str:
    return f"""
    QLabel {{
        color: {fg};
        font-family: {font_family};
        font-size: {font_size}px;
        font-weight: bold;
        {extra}
    }}
    """


# ─── Helpers ────────────────────────────────────────────────────────────
def _make_btn(text: str, style: str) -> QPushButton:
    btn = QPushButton(text)
    btn.setStyleSheet(style)
    btn.setCursor(Qt.PointingHandCursor)
    return btn


def _make_box(title: str) -> QGroupBox:
    box = QGroupBox(title)
    box.setAttribute(Qt.WA_StyledBackground, True)
    return box


def _separator() -> QFrame:
    line = QFrame()
    line.setFrameShape(QFrame.HLine)
    line.setStyleSheet("color: #555;")
    line.setFixedHeight(1)
    return line


# ─── Ventana principal ──────────────────────────────────────────────────
class MainWindow(QMainWindow):
    """Ventana principal de control del robot Staubli TX2-60L."""

    JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]

    def __init__(self, ros_node: Node | None = None):
        super().__init__()
        self.setWindowTitle("Staubli TX2-60L – GUI Control")

        # ── Nodo ROS y temporizador de spin (pueden ser None si no hay ROS) ─
        self._ros_node = ros_node
        self._ros_timer: QTimer | None = None
        self._joint_states_sub = None

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(8)

        # ── Título ────────────────────────────────────────────────────
        title = QLabel("Staubli TX2-60L")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("sans-serif", 16, QFont.Bold))
        title.setStyleSheet(f"color: {_BLANCO}; padding: 2px;")
        root.addWidget(title)

        # ── Indicador de estado ───────────────────────────────────────
        status_box = _make_box("Estado del robot")
        hb = QHBoxLayout(status_box)
        self._status_label = QLabel("● DESHABILITADO")
        self._status_label.setStyleSheet(
            _label_style(_ROJO, _FAMILIA, 15)
        )
        self._status_label.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        hb.addWidget(self._status_label)
        hb.addStretch()
        root.addWidget(status_box)

        # ── ENABLE / DISABLE ─────────────────────────────────────────
        ctrl_box = _make_box("Control")
        hb2 = QHBoxLayout(ctrl_box)
        self.btn_enable = _make_btn("ENABLE", _btn_style(_VERDE, min_h=40))
        self.btn_disable = _make_btn("DISABLE", _btn_style(_GRIS_MED, min_h=40))
        hb2.addWidget(self.btn_enable)
        hb2.addWidget(self.btn_disable)
        hb2.addStretch()
        root.addWidget(ctrl_box)

        # ── HOME ──────────────────────────────────────────────────────
        home_box = _make_box("Home / Referenciar")
        hb3 = QHBoxLayout(home_box)
        self.btn_home = _make_btn("HOME", _btn_style(_AZUL, min_w=100, min_h=40))
        hb3.addStretch()
        hb3.addWidget(self.btn_home)
        hb3.addStretch()
        root.addWidget(home_box)

        # ── Puntos de inspección ──────────────────────────────────────
        pts_box = _make_box("Puntos de inspección")
        hb4 = QHBoxLayout(pts_box)
        self.btn_p1 = _make_btn("P1", _btn_style(_GRIS_MED, min_w=64, min_h=40))
        self.btn_p2 = _make_btn("P2", _btn_style(_GRIS_MED, min_w=64, min_h=40))
        self.btn_p3 = _make_btn("P3", _btn_style(_GRIS_MED, min_w=64, min_h=40))
        hb4.addStretch()
        hb4.addWidget(self.btn_p1)
        hb4.addWidget(self.btn_p2)
        hb4.addWidget(self.btn_p3)
        hb4.addStretch()
        root.addWidget(pts_box)

        # ── Ejecutar trayectoria ──────────────────────────────────────
        traj_box = _make_box("Trayectoria")
        hb5 = QHBoxLayout(traj_box)
        self.btn_run = _make_btn(
            "EJECUTAR TRAYECTORIA",
            _btn_style(_VERDE, font_size=14, min_w=240, min_h=44, pad="10px 24px"),
        )
        hb5.addStretch()
        hb5.addWidget(self.btn_run)
        hb5.addStretch()
        root.addWidget(traj_box)

        # ── STOP ──────────────────────────────────────────────────────
        stop_box = _make_box("Seguridad")
        hb6 = QHBoxLayout(stop_box)
        self.btn_stop = _make_btn(
            "STOP",
            _btn_style(
                _ROJO,
                font_size=14,
                min_w=180,
                min_h=44,
                pad="10px 24px",
                extra=f"QPushButton:hover {{ background-color: {_ROJO_CLARO}; }}",
            ),
        )
        hb6.addStretch()
        hb6.addWidget(self.btn_stop)
        hb6.addStretch()
        root.addWidget(stop_box)

        # ── Articulaciones J1-J6 ──────────────────────────────────────
        jnt_box = _make_box("Articulaciones")
        grid = QGridLayout(jnt_box)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(8)
        grid.setColumnStretch(0, 0)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 0)
        grid.setColumnStretch(3, 1)
        self._joint_labels: dict[str, QLabel] = {}
        for idx, name in enumerate(self.JOINT_NAMES):
            row, col = divmod(idx, 2)
            lbl_name = QLabel(f"{name}:")
            lbl_name.setStyleSheet(
                _label_style(_BLANCO_HUESO, _FAM_MONO, 15, "min-width: 40px;")
            )
            lbl_name.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            lbl_val = QLabel("0.0000")
            lbl_val.setStyleSheet(
                _label_style(
                    _AMARILLO_BRILLANTE, _FAM_MONO, 16,
                    extra=(
                        f"background-color: {_NEGRO};"
                        f"border: 2px solid {_AMARILLO};"
                        "border-radius: 4px;"
                        "padding: 5px 10px;"
                        "min-width: 120px;"
                        "min-height: 30px;"
                    ),
                )
            )
            lbl_val.setAlignment(Qt.AlignCenter)
            self._joint_labels[name] = lbl_val
            grid.addWidget(lbl_name, row, col * 2)
            grid.addWidget(lbl_val,  row, col * 2 + 1)
        root.addWidget(jnt_box)

        # ── Suscripción ROS a /joint_states (si hay nodo ROS) ──────────
        if self._ros_node is not None:
            self._setup_ros()

        # ── Conexiones placeholders ───────────────────────────────────
        self.btn_enable.clicked.connect(self._on_enable)
        self.btn_disable.clicked.connect(self._on_disable)
        self.btn_home.clicked.connect(self._on_home)
        self.btn_p1.clicked.connect(lambda: self._on_punto("P1"))
        self.btn_p2.clicked.connect(lambda: self._on_punto("P2"))
        self.btn_p3.clicked.connect(lambda: self._on_punto("P3"))
        self.btn_run.clicked.connect(self._on_run)
        self.btn_stop.clicked.connect(self._on_stop)

        # ── Dimensionado: generoso y nunca menor que el contenido ─────
        self.adjustSize()
        min_w = max(640, self.sizeHint().width())
        min_h = max(720, self.sizeHint().height())
        self.setMinimumSize(min_w, min_h)
        self.resize(min_w, min_h)

    # ── ROS 2: suscripción a /joint_states ─────────────────────────────
    def _setup_ros(self) -> None:
        """Crea la suscripción y el QTimer que integra el spin con Qt."""
        self._joint_states_sub = self._ros_node.create_subscription(
            _JOINT_STATES_MSG,
            _JOINT_STATES_TOPIC,
            self._on_joint_states,
            _JOINT_STATES_QOS,
        )
        self._ros_timer = QTimer(self)
        self._ros_timer.setInterval(_ROS_SPIN_MS)
        self._ros_timer.timeout.connect(self._spin_once)
        self._ros_timer.start()
        self._ros_node.get_logger().info(
            f"Suscrito a {_JOINT_STATES_TOPIC} ({_JOINT_STATES_MSG.__name__})."
        )

    def _spin_once(self) -> None:
        """Procesa un callback ROS por tick sin bloquear la GUI."""
        if self._ros_node is not None and rclpy.ok():
            try:
                rclpy.spin_once(self._ros_node, timeout_sec=0)
            except Exception as exc:  # noqa: BLE001 - no romper la GUI
                print(f"[GUI] error en rclpy.spin_once: {exc}")

    def _on_joint_states(self, msg: JointState) -> None:
        """Actualiza J1-J6 con la posición real de cada articulación."""
        positions = dict(zip(msg.name, msg.position))
        for ros_name, gui_key in _ROS_JOINT_MAP.items():
            if ros_name in positions:
                self._joint_labels[gui_key].setText(f"{positions[ros_name]:.4f}")

    def closeEvent(self, event):  # noqa: N802 - firma de Qt
        """Detiene el spin ROS y evita callbacks tras cerrar la ventana."""
        if self._ros_timer is not None:
            self._ros_timer.stop()
            self._ros_timer = None
        super().closeEvent(event)

    # ── Callbacks placeholder (sin ROS) ───────────────────────────────
    def _set_status(self, text: str, color: str):
        self._status_label.setText(text)
        self._status_label.setStyleSheet(_label_style(color, _FAMILIA, 15))

    def _on_enable(self):
        self._set_status("● HABILITADO", _VERDE)
        print("[GUI] ENABLE presionado")

    def _on_disable(self):
        self._set_status("● DESHABILITADO", _ROJO)
        print("[GUI] DISABLE presionado")

    def _on_home(self):
        print("[GUI] HOME presionado")

    def _on_punto(self, name: str):
        print(f"[GUI] {name} presionado")

    def _on_run(self):
        print("[GUI] EJECUTAR TRAYECTORIA presionado")

    def _on_stop(self):
        self._set_status("● PARO DE EMERGENCIA", _ROJO_CLARO)
        print("[GUI] STOP presionado")


# ────────────────────────────────────────────────────────────────────────
def main():
    if not rclpy.ok():
        rclpy.init(args=None)
    node = Node("staubli_gui")
    try:
        app = QApplication(sys.argv)
        app.setStyle("Fusion")  # evita que el estilo nativo del escritorio
        app.setStyleSheet(_stylesheet())
        window = MainWindow(ros_node=node)
        window.show()
        rc = app.exec()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()