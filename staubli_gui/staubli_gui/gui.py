"""Ventana principal de la GUI del Staubli TX2-60L.

Integración ROS 2:
  1. Suscripción a /joint_states (sensor_msgs/msg/JointState) con QoS
     RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1) para mostrar en tiempo real
     las posiciones joint_1..joint_6 -> J1..J6 (radianes, 4 decimales).
  2. Servicio /controller_manager/list_controllers (~1 s) para el indicador
     de estado: "HABILITADO" si joint_trajectory_controller está 'active',
     "DESHABILITADO" en caso contrario.
  3. Acción /joint_trajectory_controller/follow_joint_trajectory (HOME):
     envía el robot a posición articular [0.0]*6 usando la misma lógica de
     goal (FollowJointTrajectory + JointTolerance) que staubli_trajectory,
     pero de forma asíncrona para no bloquear la GUI.

El bucle ROS se integra con el ciclo de eventos de Qt mediante un QTimer
que ejecuta rclpy.spin_once(...) periódicamente. ENABLE, DISABLE, P1-P3,
EJECUTAR TRAYECTORIA y STOP siguen siendo placeholders.
"""

import sys

import rclpy
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
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

# ─── ROS 2 (fase 2: indicador de estado HABILITADO/DESHABILITADO) ─────
# Consulta periódica del estado del joint_trajectory_controller mediante
# el servicio /controller_manager/list_controllers (~1 s).
_LIST_CONTROLLERS_SERVICE = "/controller_manager/list_controllers"
_LIST_CONTROLLERS_SRV = ListControllers
_JTC_CONTROLLER_NAME = "joint_trajectory_controller"
_STATUS_POLL_MS = 1000  # ~1 Hz (indicador de estado)

# ── ENABLE / DISABLE (servicio switch_controller, asíncrono) ──────────
# Activa/desactiva joint_trajectory_controller. BEST_EFFORT evita error si
# el controller ya está en el estado pedido.
_SWITCH_CONTROLLER_SERVICE = "/controller_manager/switch_controller"
_SWITCH_CONTROLLER_SRV = SwitchController
_SWITCH_STRICTNESS = SwitchController.Request.BEST_EFFORT
_SWITCH_TIMEOUT_S = 5.0

# ── HOME (acción FollowJointTrajectory, misma lógica que staubli_trajectory) ──
_HOME_ACTION = "/joint_trajectory_controller/follow_joint_trajectory"
_HOME_MSG = FollowJointTrajectory
_HOME_JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
_HOME_POSITIONS = [0.0] * 6
_HOME_DURATION_S = 10.0  # tiempo de viaje programado a la cero
# Tolerancias del goal (igual que staubli_trajectory/config/points.yaml).
_HOME_GOAL_TOL_POS = 0.01
_HOME_GOAL_TOL_VEL = 0.05
_HOME_GOAL_TOL_ACC = 0.1
_HOME_GOAL_TIME_TOL_S = 2.5
_HOME_STATUS_HOLD_MS = 2500  # mantener "HOME COMPLETADO" antes de volver al indicador
# Códigos de resultado del FollowJointTrajectoryResult (misma semántica que
# joint_trajectory_client._RESULT_CODES en staubli_trajectory).
_HOME_RESULT_CODES = {
    _HOME_MSG.Result.SUCCESSFUL: "SUCCESSFUL",
    _HOME_MSG.Result.INVALID_GOAL: "INVALID_GOAL",
    _HOME_MSG.Result.INVALID_JOINTS: "INVALID_JOINTS",
    _HOME_MSG.Result.OLD_HEADER_TIMESTAMP: "OLD_HEADER_TIMESTAMP",
    _HOME_MSG.Result.PATH_TOLERANCE_VIOLATED: "PATH_TOLERANCE_VIOLATED",
    _HOME_MSG.Result.GOAL_TOLERANCE_VIOLATED: "GOAL_TOLERANCE_VIOLATED",
}


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
        self._list_controllers_client = None
        self._status_timer: QTimer | None = None
        self._switch_controller_client = None
        self._switch_pending = False
        self._home_client: ActionClient | None = None
        self._home_goal_handle = None
        self._home_active = False

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

    # ── ROS 2: suscripción a /joint_states + estado del controller ────
    def _setup_ros(self) -> None:
        """Crea la suscripción, el client de list_controllers y los QTimers."""
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

        # Estado del joint_trajectory_controller (list_controllers).
        self._list_controllers_client = self._ros_node.create_client(
            _LIST_CONTROLLERS_SRV, _LIST_CONTROLLERS_SERVICE
        )
        self._status_timer = QTimer(self)
        self._status_timer.setInterval(_STATUS_POLL_MS)
        self._status_timer.timeout.connect(self._poll_controller_status)
        self._status_timer.start()
        self._poll_controller_status()  # consulta inmediata inicial

        # HOME: action client del joint_trajectory_controller (async, no bloquea).
        self._home_client = ActionClient(
            self._ros_node, _HOME_MSG, _HOME_ACTION
        )

        # ENABLE / DISABLE: client del servicio switch_controller.
        self._switch_controller_client = self._ros_node.create_client(
            _SWITCH_CONTROLLER_SRV, _SWITCH_CONTROLLER_SERVICE
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

    # ── ROS 2: estado del joint_trajectory_controller ─────────────────
    def _poll_controller_status(self) -> None:
        """Lanza una consulta asíncrona a /controller_manager/list_controllers."""
        if self._list_controllers_client is None or not rclpy.ok():
            return
        if not self._list_controllers_client.service_is_ready():
            return
        try:
            future = self._list_controllers_client.call_async(
                _LIST_CONTROLLERS_SRV.Request()
            )
            future.add_done_callback(self._on_controller_status_response)
        except Exception as exc:  # noqa: BLE001 - no romper la GUI
            print(f"[GUI] error consultando estado del controller: {exc}")

    def _on_controller_status_response(self, future) -> None:
        """Actualiza el indicador según el estado de joint_trajectory_controller."""
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - no romper la GUI
            print(f"[GUI] fallo en list_controllers: {exc}")
            return
        estado = None
        for ctrl in response.controller:
            if ctrl.name == _JTC_CONTROLLER_NAME:
                estado = ctrl.state
                break
        if estado == "active":
            self._set_status("● HABILITADO", _VERDE)
        else:
            self._set_status("● DESHABILITADO", _ROJO)

    def closeEvent(self, event):  # noqa: N802 - firma de Qt
        """Detiene los timers ROS y evita callbacks tras cerrar la ventana."""
        if self._home_active and self._home_goal_handle is not None:
            try:
                self._home_goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001 - no romper el cierre
                pass
        if self._ros_timer is not None:
            self._ros_timer.stop()
            self._ros_timer = None
        if self._status_timer is not None:
            self._status_timer.stop()
            self._status_timer = None
        super().closeEvent(event)

    # ── Callbacks placeholder (sin ROS) ───────────────────────────────
    def _set_status(self, text: str, color: str):
        self._status_label.setText(text)
        self._status_label.setStyleSheet(_label_style(color, _FAMILIA, 15))

    # ── ENABLE / DISABLE (switch_controller, asíncrono) ───────────────
    def _on_enable(self):
        self._switch_controller(activate=True)

    def _on_disable(self):
        self._switch_controller(activate=False)

    def _switch_controller(self, activate: bool) -> None:
        """Activa/desactiva joint_trajectory_controller sin bloquear la GUI."""
        if self._ros_node is None or self._switch_controller_client is None:
            print("[GUI] switch_controller sin ROS (placeholder)")
            return
        if self._switch_pending:
            return
        if not self._switch_controller_client.service_is_ready():
            self._set_status("● CONTROLLER: SERVICIO NO DISPONIBLE", _ROJO)
            self._ros_node.get_logger().error(
                f"{_SWITCH_CONTROLLER_SERVICE} no disponible."
            )
            return

        request = _SWITCH_CONTROLLER_SRV.Request()
        if activate:
            request.activate_controllers = [_JTC_CONTROLLER_NAME]
        else:
            request.deactivate_controllers = [_JTC_CONTROLLER_NAME]
        request.strictness = _SWITCH_STRICTNESS
        request.activate_asap = False
        request.timeout = Duration(seconds=_SWITCH_TIMEOUT_S).to_msg()

        accion = "ENABLE" if activate else "DISABLE"
        self._switch_pending = True
        self.btn_enable.setEnabled(False)
        self.btn_disable.setEnabled(False)
        self._set_status(
            f"● {accion}: {'ACTIVANDO' if activate else 'DESACTIVANDO'}...", _AZUL
        )
        self._ros_node.get_logger().info(
            f"{accion}: switch_controller "
            f"{'activate' if activate else 'deactivate'} {_JTC_CONTROLLER_NAME}..."
        )
        try:
            future = self._switch_controller_client.call_async(request)
            future.add_done_callback(
                lambda fut: self._on_switch_controller_response(fut, accion)
            )
        except Exception as exc:  # noqa: BLE001 - no romper la GUI
            self._finish_switch(accion, False, str(exc))

    def _on_switch_controller_response(self, future, accion: str) -> None:
        """Respuesta del switch_controller: ok + mensaje de error si falla."""
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - no romper la GUI
            self._finish_switch(accion, False, str(exc))
            return
        self._finish_switch(accion, bool(response.ok), response.message)

    def _finish_switch(self, accion: str, ok: bool, message: str) -> None:
        """Restaura botones y refleja el resultado de ENABLE/DISABLE."""
        self._switch_pending = False
        self.btn_enable.setEnabled(True)
        self.btn_disable.setEnabled(True)
        if self._ros_node is not None:
            logger = self._ros_node.get_logger()
            (logger.info if ok else logger.error)(
                f"{accion}: {'OK' if ok else 'FALLÓ'} ({message})"
            )
        if ok:
            if accion == "ENABLE":
                self._set_status("● HABILITADO", _VERDE)
            else:
                self._set_status("● DESHABILITADO", _ROJO)
            self._poll_controller_status()  # confirmar con el estado real
        else:
            self._set_status(f"● {accion} FALLÓ: {message}", _ROJO)

    def _on_home(self):
        """Envía el robot a la posición cero (HOME) sin bloquear la GUI."""
        if self._home_active:
            return
        if self._home_client is None or self._list_controllers_client is None:
            print("[GUI] HOME presionado (sin ROS: placeholder)")
            return

        self._home_active = True
        self.btn_home.setEnabled(False)  # evita goals simultáneos
        self._pause_status_poll()
        self._set_status("● HOME: COMPROBANDO CONTROLLER...", _AZUL)

        # Antes de enviar: exigir joint_trajectory_controller 'active'.
        if not self._list_controllers_client.service_is_ready():
            self._finish_home(
                "● HOME: NO SE PUEDE VERIFICAR EL CONTROLLER", _ROJO
            )
            return
        try:
            future = self._list_controllers_client.call_async(
                _LIST_CONTROLLERS_SRV.Request()
            )
            future.add_done_callback(self._on_home_controller_check)
        except Exception as exc:  # noqa: BLE001 - no romper la GUI
            self._finish_home(f"● HOME: ERROR AL VERIFICAR ({exc})", _ROJO)

    def _on_home_controller_check(self, future) -> None:
        """Solo envía el goal si joint_trajectory_controller está 'active'."""
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - no romper la GUI
            self._finish_home(f"● HOME: ERROR AL VERIFICAR ({exc})", _ROJO)
            return

        estado = None
        for ctrl in response.controller:
            if ctrl.name == _JTC_CONTROLLER_NAME:
                estado = ctrl.state
                break
        if estado != "active":
            self._finish_home(
                f"● HOME: CONTROLLER {estado or 'NO ENCONTRADO'} (se requiere active)",
                _ROJO,
            )
            return

        if not self._home_client.server_is_ready():
            self._finish_home("● HOME: ACTION SERVER NO DISPONIBLE", _ROJO)
            return

        self._set_status("MOVIENDO A HOME...", _AZUL)
        goal = self._build_home_goal()
        self._ros_node.get_logger().info(
            f"HOME: enviando goal a posición cero {_HOME_POSITIONS}."
        )
        goal_future = self._home_client.send_goal_async(goal)
        goal_future.add_done_callback(self._on_home_goal_response)

    # ── HOME: construcción del goal y seguimiento asíncrono ───────────
    def _build_home_goal(self) -> FollowJointTrajectory.Goal:
        """Goal HOME: trayectoria + tolerancias (igual que staubli_trajectory)."""
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in _HOME_POSITIONS]
        point.time_from_start = Duration(seconds=_HOME_DURATION_S).to_msg()

        trajectory = JointTrajectory()
        trajectory.joint_names = list(_HOME_JOINT_NAMES)
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        for name in _HOME_JOINT_NAMES:
            tol = JointTolerance()
            tol.name = name
            tol.position = _HOME_GOAL_TOL_POS
            tol.velocity = _HOME_GOAL_TOL_VEL
            tol.acceleration = _HOME_GOAL_TOL_ACC
            goal.goal_tolerance.append(tol)
        goal.goal_time_tolerance = Duration(
            seconds=_HOME_GOAL_TIME_TOL_S
        ).to_msg()
        return goal

    def _on_home_goal_response(self, future) -> None:
        """El controller acepta/rechaza el goal; espera el resultado real."""
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001 - no romper la GUI
            self._finish_home(f"● HOME: ERROR AL ENVIAR ({exc})", _ROJO)
            return
        if not goal_handle.accepted:
            self._finish_home("● HOME: GOAL RECHAZADO", _ROJO)
            return

        self._home_goal_handle = goal_handle
        self._ros_node.get_logger().info(
            "HOME: goal ACEPTADO; esperando resultado de ejecución."
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_home_result)

    def _on_home_result(self, future) -> None:
        """Comprueba el estado final del goal (no basta con haberlo enviado)."""
        try:
            wrapped = future.result()
            error_code = getattr(wrapped.result, "error_code", None)
        except Exception as exc:  # noqa: BLE001 - no romper la GUI
            self._finish_home(f"● HOME: ERROR ({exc})", _ROJO)
            return

        if error_code == _HOME_MSG.Result.SUCCESSFUL:
            self._finish_home("● HOME COMPLETADO", _VERDE)
        else:
            nombre = _HOME_RESULT_CODES.get(
                error_code, f"desconocido ({error_code})"
            )
            self._finish_home(f"● HOME FALLÓ: {nombre}", _ROJO)

    def _finish_home(self, status_text: str, color: str) -> None:
        """Restaura botón y sondeo de estado al terminar (o fallar) HOME."""
        self._home_active = False
        self._home_goal_handle = None
        self.btn_home.setEnabled(True)
        self._set_status(status_text, color)
        QTimer.singleShot(_HOME_STATUS_HOLD_MS, self._resume_status_poll)

    def _pause_status_poll(self) -> None:
        """Evita que el indicador (1 Hz) pise el estado de HOME en curso."""
        if self._status_timer is not None:
            self._status_timer.stop()

    def _resume_status_poll(self) -> None:
        """Reanuda el indicador (salvo que haya otro HOME en curso)."""
        if self._status_timer is not None and not self._home_active:
            self._status_timer.start()
            self._poll_controller_status()

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