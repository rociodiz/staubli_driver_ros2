"""Ventana principal de la GUI del Staubli TX2-60L.

Integración ROS 2:
  1. Suscripción a /joint_states (sensor_msgs/msg/JointState) con QoS
     RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1) para mostrar en tiempo real
     las posiciones joint_1..joint_6 -> J1..J6 (radianes, 4 decimales).
  2. Servicio /controller_manager/list_controllers (~1 s) para el indicador
     de estado y el modo actual (TRAYECTORIA/MANUAL/SEGURO) basado en el
     estado real de joint_trajectory_controller y forward_velocity_controller.
  3. Servicio /controller_manager/switch_controller (STRICT) para cambiar de
     modo de forma atómica y verificar el resultado sin terminal manual.
  3. Acción /joint_trajectory_controller/follow_joint_trajectory (HOME y
     P1-P3): envía goals de un solo waypoint (posición articular) usando la
     misma lógica de goal (FollowJointTrajectory + JointTolerance) que
     staubli_trajectory, pero de forma asíncrona para no bloquear la GUI.
     Las IK de P1/P2/P3 se reutilizan de staubli_trajectory (points.yaml +
     TrajectoryBuilder); no se duplican coordenadas ni IK aquí.

El bucle ROS se integra con el ciclo de eventos de Qt mediante un QTimer
que ejecuta rclpy.spin_once(...) periódicamente.

Modo de control EXCLUYENTE: las acciones HOME, P1-P3, MOVER A PUNTO y
EJECUTAR TRAYECTORIA exigen joint_trajectory_controller activo (modo
TRAYECTORIA); el MANDO VIRTUAL exige forward_velocity_controller (modo
MANUAL). Nunca pueden estar ambos activos: la GUI cambia los
controladores por /controller_manager/switch_controller (STRICT, atómico)
antes de cada acción, verifica el resultado y avisa si falla, de modo que
el usuario no necesita tocar la terminal. El joint_state_broadcaster queda
siempre activo. En el arranque la GUI corrige automáticamente un posible
conflicto (JTC+FVC activos) volviendo a TRAYECTORIA.

DO STOP detiene el movimiento en curso: con el control manual activo envía
Twist=0 y vuelve a TRAYECTORIA; con un goal activo lo cancela. El panel
"Mensajes" refleja en tiempo real los eventos del sistema.

   Cámara eye-in-hand: suscripción directa a /camera/image_raw
   (sensor_msgs/msg/Image, QoS sensor_data) y conversión de canal
   'rgb8' a QImage (QtGui, sin OpenCV/cv_bridge/numpy). El callback ROS
   corre en el hilo de eventos de Qt (spin_once desde QTimer), así que
   actualiza el visor directamente. La imagen se escala con Qt manteniendo
   la relación de aspecto; solo se re-escala cuando cambia el tamaño del
   visor (no en cada frame), para no penalizar el spin a 30 Hz.
   4. Control manual (FRAME): seis pares de botones +/− (X, Y, Z, RX, RY,
      RZ) que generan un Twist cartesiano normalizado que la GUI publica en
      /cmd_vel SOLO con el control manual ACTIVADO (botón ACTIVAR MANUAL ->
      cambio a forward_velocity_controller). Al desactivar (VOLVER A
      TRAYECTORIA) envía un Twist=0 y restaura el joint_trajectory_controller.
      NUNCA calcula IK ni velocidades articulares (lo hace el servo).
"""

import sys
import time

import rclpy
from action_msgs.srv import CancelGoal
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from PySide6.QtCore import QSize, Qt, QTimer
from PySide6.QtGui import QColor, QFont, QImage, QPixmap, QTextCharFormat, QTextCursor
from PySide6.QtWidgets import (
    QApplication,
    QDialog,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from .controller_manager import (
    ControllerManager,
    FORWARD_VELOCITY_CONTROLLER,
    JOINT_TRAJECTORY_CONTROLLER,
    derive_mode,
    switch_plan,
)
from .frame_control import FrameControlPanel
from .user_points import (
    UserPoint,
    UserPointStore,
    rpy_degrees_to_quaternion_xyzw,
    user_point_from_joint_position,
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

# ── ROS 2 (cámara eye-in-hand, fase 3: visor de vídeo) ───────────────
# El bridge (launch_gazebo.launch.py) publica /camera/image_raw como
# sensor_msgs/msg/Image. QoS sensor_data (BEST_EFFORT, VOLATILE): compatible
# con ros_gz_bridge y sin acumular frames si el visor va más lento.
_CAMERA_TOPIC = "/camera/image_raw"
_CAMERA_MSG = Image
_CAMERA_QOS = qos_profile_sensor_data
_CAMERA_RGB8 = "rgb8"  # el sensor gz publica R8G8B8 -> 'rgb8' (sin cv_bridge)

# ─── ROS 2 (fase 2: indicador de estado de los controladores) ─────────
# Consulta periódica del estado de los controladores mediante el servicio
# /controller_manager/list_controllers (~1 s). Los modos TRAYECTORIA
# (joint_trajectory_controller) y MANUAL (forward_velocity_controller)
# son EXCLUYENTES; joint_state_broadcaster queda siempre activo.
_STATUS_POLL_MS = 1000  # ~1 Hz (indicador de estado y del modo)

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

# ─── ROS 2 (control manual, fase 4: publicación de /cmd_vel) ──────────
# La GUI publica geometry_msgs/Twist en /cmd_vel SOLO con el control
# manual activado. El control NO calcula IK ni velocidades articulares: ese
# trabajo es de staubli_joystick_servo (Jacobi + DLS). QoS RELIABLE depth 10,
# compatible con la suscripción del servo (default).
_CMD_VEL_TOPIC = "/cmd_vel"
_CMD_VEL_MSG = Twist
_CMD_VEL_QOS_DEPTH = 10

# ── Panel de mensajes ──────────────────────────────────────────────────
_LOG_MAX_BLOCKS = 200  # log acotado (QPlainTextEdit descarta bloques viejos)

# ── P1/P2/P3 (waypoints individuales; IK reutilizada de staubli_trajectory) ──
# points.yaml sigue siendo la única fuente de verdad: la GUI solo lee las
# posiciones articulares ya resueltas por TrajectoryBuilder (sin IK propia).
_POINTS_PKG = "staubli_trajectory"  # paquete propietario de config/points.yaml
_POINTS_RELATIVE_CONFIG = ("config", "points.yaml")
_POINT_DURATION_S = 10.0  # duración de cada waypoint P1/P2/P3 (configurable)
_POINT_BTN_ATTR = {"P1": "btn_p1", "P2": "btn_p2", "P3": "btn_p3"}
# Nombre del waypoint en points.yaml al que corresponde cada botón P.
_POINT_WAYPOINT_NAMES = {
    "P1": "inspection_fronto_superior",
    "P2": "inspection_izquierda",
    "P3": "inspection_derecha",
}

# ── Trayectoria definida por el usuario ─────────────────────────────────
# Paso temporal entre waypoints (5 s, 10 s, 15 s, ...). No usa points.yaml.
_USER_TRAJ_STEP_S = 5.0


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


# ─── Visor de vídeo ────────────────────────────────────────────────────
class CameraView(QLabel):
    """Etiqueta que muestra un QImage escalado manteniendo la relación de
    aspecto dentro del espacio disponible del widget.

    Cada frame entrante se re-escala (Qt.KeepAspectRatio + SmoothTransformation)
    y actualiza el QPixmap; la geometría objetivo solo se recalcula cuando
    cambia el tamaño del visor, no en cada frame.
    """

    def __init__(self, placeholder: str = "Sin imagen") -> None:
        super().__init__(placeholder)
        self._frame: QImage | None = None
        self._scaled_size: tuple[int, int] | None = None

    def set_frame(self, image: QImage) -> None:
        """Reemplaza el frame actual y actualiza el visor con el nuevo frame."""
        self._frame = image
        self._render()

    def resizeEvent(self, event) -> None:  # noqa: N802 - firma de Qt
        super().resizeEvent(event)
        self._render()

    def _render(self) -> None:
        if self._frame is None or self._frame.isNull():
            return
        size = (self.width(), self.height())
        if size[0] <= 0 or size[1] <= 0:
            return
        # Solo se recalcula la geometría objetivo si cambió el tamaño del visor.
        if size != self._scaled_size:
            self._scaled_size = size
        scaled = self._frame.scaled(
            QSize(self._scaled_size[0], self._scaled_size[1]),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self.setPixmap(QPixmap.fromImage(scaled))


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
        self._ctrl: ControllerManager | None = None  # cliente de /controller_manager
        self._status_timer: QTimer | None = None
        self._switch_busy = False  # hay un cambio de controladores en curso
        self._after_switch = None  # callable(ok, msg) al terminar ese cambio
        self._action_client: ActionClient | None = None
        self._motion_goal_handle = None
        self._motion_active = False  # exclusividad HOME + P1-P3 + TRAYECTORIA
        self._stop_requested = False  # STOP: distingue cancelación vs. éxito
        self._motion_button = None
        self._point_plan = None
        self._point_qs: dict = {}
        self._point_ready = False
        self._store: UserPointStore | None = None
        self._user_trajectory: list[str] = []  # nombres de UserPoint en orden
        self._joints_now: dict | None = None  # última lectura /joint_states
        self._user_kinematics = None
        self._camera_sub = None  # suscripción a /camera/image_raw
        self._camera_view: CameraView | None = None  # visor de vídeo
        self._warned_encoding: str | None = None  # encoding ignorado (1 aviso)
        self._cmd_vel_pub = None  # publisher de /cmd_vel (control manual)
        self._frame_panel: FrameControlPanel | None = None  # control FRAME
        self._manual_active = False  # solo se publica /cmd_vel si está activo

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 8, 10, 8)
        root.setSpacing(10)

        # ── Título (cruza toda la ventana) ─────────────────────────────
        title = QLabel("Staubli TX2-60L")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("sans-serif", 15, QFont.Bold))
        title.setStyleSheet(f"color: {_BLANCO}; padding: 0px;")
        root.addWidget(title)

        # ── Dos columnas principales (~50/50, apaisado) ────────────────
        cols = QHBoxLayout()
        cols.setSpacing(12)
        root.addLayout(cols)

        # Columna izquierda: control manual + visión.
        left = QVBoxLayout()
        left.setSpacing(10)
        cols.addLayout(left, 1)

        # Columna derecha: operación automática + estado.
        right = QVBoxLayout()
        right.setSpacing(10)
        cols.addLayout(right, 1)

        # ── Barra de control superior (compacta, altura fija) ─────────
        top_box = _make_box("Control del robot")
        top_box.setFixedHeight(58)
        hb = QHBoxLayout(top_box)
        hb.setContentsMargins(10, 2, 10, 4)
        hb.setSpacing(12)
        self._status_label = QLabel("● CONECTANDO...")
        self._status_label.setStyleSheet(_label_style(_GRIS_MED, _FAMILIA, 13))
        self._status_label.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        hb.addWidget(self._status_label)
        hb.addSpacing(18)
        self.btn_enable = _make_btn("ENABLE", _btn_style(_VERDE, min_h=30))
        self.btn_disable = _make_btn("DISABLE", _btn_style(_GRIS_MED, min_h=30))
        hb.addWidget(self.btn_enable)
        hb.addWidget(self.btn_disable)
        hb.addStretch()
        self.btn_home = _make_btn("HOME", _btn_style(_AZUL, min_w=110, min_h=30))
        hb.addWidget(self.btn_home)
        right.addWidget(top_box)

        # ── CÁMARA (zona principal: imagen + control manual abajo) ────
        cam_box = _make_box("Cámara")
        cam_v = QVBoxLayout(cam_box)
        cam_v.setContentsMargins(4, 4, 4, 4)
        self._camera_view = CameraView(
            "Cámara: sin imagen\n(esperando /camera/image_raw)"
        )
        self._camera_view.setAlignment(Qt.AlignCenter)
        self._camera_view.setWordWrap(True)
        self._camera_view.setStyleSheet(
            f"background-color: {_NEGRO}; border-radius: 4px;"
            f" color: {_GRIS_CLR};"
            f" font-family: {_FAMILIA}; font-size: 12px;"
        )
        self._camera_view.setMinimumSize(240, 160)
        cam_v.addWidget(self._camera_view)
        left.addWidget(cam_box, 3)

        # ── CONTROL MANUAL (mando virtual: zona principal) ────────────
        # Solo produce /cmd_vel cuando está activado (el switch de
        # controller JTC -> forward_velocity_controller queda pendiente).
        self._frame_panel = FrameControlPanel()
        self._frame_panel.toggled.connect(self._on_manual_toggle)
        left.addWidget(self._frame_panel, 4)

        # ── GUARDAR POSICIÓN ACTUAL (captura J1-J6 reales como punto) ────
        self.btn_capture_pos = _make_btn(
            "GUARDAR POSICIÓN ACTUAL",
            _btn_style(
                _AMARILLO,
                fg=_GRIS_OSC,
                font_size=12,
                min_w=230,
                min_h=32,
                pad="6px 14px",
            ),
        )
        capt_row = QHBoxLayout()
        capt_row.addStretch()
        capt_row.addWidget(self.btn_capture_pos)
        capt_row.addStretch()
        left.addLayout(capt_row)

        # ── STOP (visible e importante, integrado bajo el control manual) ───
        self.btn_stop = _make_btn(
            "STOP",
            _btn_style(
                _ROJO,
                font_size=15,
                min_w=170,
                min_h=36,
                pad="8px 20px",
                extra=f"QPushButton:hover {{ background-color: {_ROJO_CLARO}; }}",
            ),
        )
        stop_row = QHBoxLayout()
        stop_row.addStretch()
        stop_row.addWidget(self.btn_stop)
        stop_row.addStretch()
        left.addLayout(stop_row)

        # ── PUNTOS DE INSPECCIÓN (P1 / P2 / P3) ────────────────────────
        insp_box = _make_box("Puntos de inspección")
        v_puntos = QVBoxLayout(insp_box)
        v_puntos.setSpacing(4)
        hb_puntos = QHBoxLayout()
        hb_puntos.setSpacing(8)
        self.btn_p1 = _make_btn("P1", _btn_style(_GRIS_MED, min_w=54, min_h=30))
        self.btn_p2 = _make_btn("P2", _btn_style(_GRIS_MED, min_w=54, min_h=30))
        self.btn_p3 = _make_btn("P3", _btn_style(_GRIS_MED, min_w=54, min_h=30))
        hb_puntos.addWidget(self.btn_p1)
        hb_puntos.addWidget(self.btn_p2)
        hb_puntos.addWidget(self.btn_p3)
        v_puntos.addLayout(hb_puntos)
        right.addWidget(insp_box)
        # ── PUNTOS DEL USUARIO (lista + acciones) ─────────────────────
        user_box = _make_box("Puntos del usuario")
        v_user = QVBoxLayout(user_box)
        v_user.setSpacing(6)
        self._points_list = QListWidget()
        self._points_list.setStyleSheet(
            f"QListWidget {{ background-color: {_NEGRO}; color: {_BLANCO_HUESO};"
            f" border: 1px solid {_GRIS_MED}; border-radius: 4px;"
            f" font-family: {_FAM_MONO}; font-size: 12px; }}"
            f"QListWidget::item {{ padding: 3px 6px; }}"
            f"QListWidget::item:selected {{ background-color: {_AZUL}; }}"
        )
        self._points_list.setMinimumHeight(56)
        v_user.addWidget(self._points_list)
        up_btns = QGridLayout()
        up_btns.setHorizontalSpacing(8)
        up_btns.setVerticalSpacing(6)
        up_btns.setColumnStretch(4, 1)
        self.btn_new_point = _make_btn(
            "NUEVO PUNTO", _btn_style(_AZUL, font_size=11, min_h=28, pad="4px 10px")
        )
        self.btn_edit_point = _make_btn(
            "EDITAR", _btn_style(_GRIS_MED, font_size=11, min_h=28, pad="4px 10px")
        )
        self.btn_delete_point = _make_btn(
            "ELIMINAR", _btn_style(_ROJO, font_size=11, min_h=28, pad="4px 10px")
        )
        self.btn_move_point = _make_btn(
            "MOVER A PUNTO", _btn_style(_VERDE, font_size=11, min_h=28, pad="4px 10px")
        )
        self.btn_add_to_traj = _make_btn(
            "AÑADIR A TRAYECTORIA",
            _btn_style(_AZUL, font_size=11, min_h=28, pad="4px 10px"),
        )
        up_btns.addWidget(self.btn_new_point, 0, 0)
        up_btns.addWidget(self.btn_edit_point, 0, 1)
        up_btns.addWidget(self.btn_delete_point, 0, 2)
        up_btns.addWidget(self.btn_move_point, 1, 0)
        up_btns.addWidget(self.btn_add_to_traj, 1, 1, 1, 2)
        v_user.addLayout(up_btns)
        self.btn_edit_point.setEnabled(False)
        self.btn_delete_point.setEnabled(False)
        self.btn_move_point.setEnabled(False)
        self.btn_add_to_traj.setEnabled(False)
        right.addWidget(user_box, 1)

        # ── TRAYECTORIA DEFINIDA (orden editable) + EJECUTAR ──────────
        traj_box = _make_box("Trayectoria definida")
        v_traj = QVBoxLayout(traj_box)
        v_traj.setSpacing(6)
        self._trajectory_list = QListWidget()
        self._trajectory_list.setStyleSheet(
            f"QListWidget {{ background-color: {_NEGRO}; color: {_BLANCO_HUESO};"
            f" border: 1px solid {_GRIS_MED}; border-radius: 4px;"
            f" font-family: {_FAM_MONO}; font-size: 12px; }}"
            f"QListWidget::item {{ padding: 3px 6px; }}"
            f"QListWidget::item:selected {{ background-color: {_AZUL}; }}"
        )
        self._trajectory_list.setMinimumHeight(56)
        v_traj.addWidget(self._trajectory_list)
        tuv_btns = QHBoxLayout()
        tuv_btns.setSpacing(8)
        self.btn_traj_up = _make_btn(
            "↑", _btn_style(_GRIS_MED, font_size=12, min_h=28, min_w=38, pad="2px 8px")
        )
        self.btn_traj_down = _make_btn(
            "↓", _btn_style(_GRIS_MED, font_size=12, min_h=28, min_w=38, pad="2px 8px")
        )
        self.btn_traj_remove = _make_btn(
            "QUITAR", _btn_style(_ROJO, font_size=11, min_h=28, pad="4px 10px")
        )
        self.btn_traj_clear = _make_btn(
            "LIMPIAR", _btn_style(_GRIS_MED, font_size=11, min_h=28, pad="4px 10px")
        )
        tuv_btns.addWidget(self.btn_traj_up)
        tuv_btns.addWidget(self.btn_traj_down)
        tuv_btns.addWidget(self.btn_traj_remove)
        tuv_btns.addWidget(self.btn_traj_clear)
        tuv_btns.addStretch()
        self.btn_run = _make_btn(
            "EJECUTAR TRAYECTORIA",
            _btn_style(_VERDE, font_size=12, min_w=170, min_h=32, pad="6px 14px"),
        )
        v_traj.addLayout(tuv_btns)
        tuv_run = QHBoxLayout()
        tuv_run.addStretch()
        tuv_run.addWidget(self.btn_run)
        v_traj.addLayout(tuv_run)
        self.btn_traj_up.setEnabled(False)
        self.btn_traj_down.setEnabled(False)
        self.btn_traj_remove.setEnabled(False)
        self.btn_traj_clear.setEnabled(False)
        right.addWidget(traj_box, 1)

        # ── ESTADO / ARTICULACIONES (información secundaria) ──────────
        jnt_box = _make_box("Estado / Articulaciones")
        grid = QGridLayout(jnt_box)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(6)
        grid.setColumnStretch(0, 0)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 0)
        grid.setColumnStretch(3, 1)
        grid.setColumnStretch(4, 0)
        grid.setColumnStretch(5, 1)
        self._joint_labels: dict[str, QLabel] = {}
        for idx, name in enumerate(self.JOINT_NAMES):
            row, col = divmod(idx, 3)
            lbl_name = QLabel(f"{name}:")
            lbl_name.setStyleSheet(
                _label_style(_BLANCO_HUESO, _FAM_MONO, 13, "min-width: 40px;")
            )
            lbl_name.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            lbl_val = QLabel("0.0000")
            lbl_val.setStyleSheet(
                _label_style(
                    _AMARILLO_BRILLANTE, _FAM_MONO, 14,
                    extra=(
                        f"background-color: {_NEGRO};"
                        f"border: 2px solid {_AMARILLO};"
                        "border-radius: 4px;"
                        "padding: 3px 8px;"
                        "min-width: 100px;"
                        "min-height: 22px;"
                    ),
                )
            )
            lbl_val.setAlignment(Qt.AlignCenter)
            self._joint_labels[name] = lbl_val
            grid.addWidget(lbl_name, row, col * 2)
            grid.addWidget(lbl_val,  row, col * 2 + 1)

        right.addWidget(jnt_box)

        # ── Mensajes / log del sistema (protagonismo reducido) ───────
        log_box = _make_box("Mensajes")
        log_v = QVBoxLayout(log_box)
        log_v.setSpacing(4)
        log_h = QHBoxLayout()
        hint = QLabel("Sistema")
        hint.setStyleSheet(_label_style(_GRIS_CLR, _FAMILIA, 11))
        log_h.addWidget(hint)
        log_h.addStretch()
        self._btn_log_clear = _make_btn(
            "LIMPIAR",
            _btn_style(_GRIS_MED, font_size=10, min_h=22, pad="2px 12px"),
        )
        log_h.addWidget(self._btn_log_clear)
        log_v.addLayout(log_h)
        self._log_edit = QPlainTextEdit()
        self._log_edit.setReadOnly(True)
        self._log_edit.setMaximumBlockCount(_LOG_MAX_BLOCKS)
        self._log_edit.setStyleSheet(
            "QPlainTextEdit {"
            f" background-color: {_NEGRO};"
            f" color: {_GRIS_CLR};"
            f" border: 1px solid {_GRIS_MED};"
            " border-radius: 4px;"
            f" font-family: {_FAM_MONO};"
            " font-size: 12px;"
            "}"
        )
        self._log_edit.setMinimumHeight(64)
        log_v.addWidget(self._log_edit)

        # Mensajes: sección final de la columna derecha (crece con la ventana).
        right.addWidget(log_box, 1)

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
        self._btn_log_clear.clicked.connect(self._clear_log)
        self._points_list.currentItemChanged.connect(self._update_point_buttons)
        self.btn_new_point.clicked.connect(self._new_point)
        self.btn_edit_point.clicked.connect(self._edit_point)
        self.btn_delete_point.clicked.connect(self._delete_point)
        self.btn_move_point.clicked.connect(self._move_to_point)
        self.btn_add_to_traj.clicked.connect(self._on_add_to_trajectory)
        self.btn_capture_pos.clicked.connect(self._on_capture_position)
        self._trajectory_list.currentItemChanged.connect(
            self._update_trajectory_buttons
        )
        self.btn_traj_up.clicked.connect(self._on_traj_move_up)
        self.btn_traj_down.clicked.connect(self._on_traj_move_down)
        self.btn_traj_remove.clicked.connect(self._on_traj_remove)
        self.btn_traj_clear.clicked.connect(self._on_traj_clear)

        # ── Almacén de puntos del usuario (independiente de ROS) ───────
        self._load_user_points()

        # ── Dimensionado: apaisado (~16:9), mínimo con suelo ancho ────
        # El contenido no debe empujar la ventana en vertical: el ancho
        # disponible lo reparten las dos columnas (~50/50).
        mh = self.minimumSizeHint()
        min_w = max(1120, mh.width())
        min_h = max(640, mh.height())
        self.setMinimumSize(min_w, min_h)
        self.resize(max(min_w, 1280), max(min_h, 850))

    # ── ROS 2: suscripción a /joint_states + estado del controller ────
    def _setup_ros(self) -> None:
        """Crea la suscripción, el client de list_controllers y los QTimers."""
        self._log("ok", "Sistema iniciado (ROS listo)")
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
        self._log("info", f"Suscrito a {_JOINT_STATES_TOPIC}")

        # Cámara eye-in-hand: suscripción directa a /camera/image_raw.
        if self._camera_view is not None:
            self._camera_sub = self._ros_node.create_subscription(
                _CAMERA_MSG,
                _CAMERA_TOPIC,
                self._on_camera_image,
                _CAMERA_QOS,
            )
            self._ros_node.get_logger().info(
                f"Suscrito a {_CAMERA_TOPIC} ({_CAMERA_MSG.__name__})."
            )
            self._log("info", f"Suscrito a {_CAMERA_TOPIC}")

        # Estados de los controladores (list_controllers + switch_controller):
        # el modo TRAYECTORIA/MANUAL es excluyente y se gestiona aquí.
        self._ctrl = ControllerManager(self._ros_node)
        self._status_timer = QTimer(self)
        self._status_timer.setInterval(_STATUS_POLL_MS)
        self._status_timer.timeout.connect(self._poll_controller_status)
        self._status_timer.start()
        self._poll_controller_status()  # consulta inmediata inicial

        # Acción del joint_trajectory_controller (HOME, P1-P3 y trayectoria; async).
        self._action_client = ActionClient(
            self._ros_node, _HOME_MSG, _HOME_ACTION
        )

        # IK/config de points.yaml -> posiciones articulares de P1/P2/P3.
        self._load_point_plan()

        # Control manual: publish de /cmd_vel (solo cuando está activo).
        if self._frame_panel is not None:
            self._cmd_vel_pub = self._ros_node.create_publisher(
                _CMD_VEL_MSG, _CMD_VEL_TOPIC, _CMD_VEL_QOS_DEPTH
            )
            self._ros_node.get_logger().info(
                f"Publicando {_CMD_VEL_TOPIC} ({_CMD_VEL_MSG.__name__})."
            )
            self._log("info", f"Publicando {_CMD_VEL_TOPIC}")

    def _spin_once(self) -> None:
        """Procesa un callback ROS por tick sin bloquear la GUI.

        Si ROS se ha apagado (p.ej. SIGINT/Ctrl+C desde ros2 launch, que hace
        que rclpy cierre su contexto) la GUI se cierra sola: si no, quedaría
        como proceso huérfano y el siguiente arranque crearía un nodo
        /staubli_gui duplicado.
        """
        if self._ros_node is None:
            return
        if not rclpy.ok():
            self._close_for_shutdown()
            return
        try:
            rclpy.spin_once(self._ros_node, timeout_sec=0)
        except Exception as exc:  # noqa: BLE001 - spin_once lanza al apagarse
            self._ros_node.get_logger().warn(f"rclpy detenido: {exc}")
            self._close_for_shutdown()
            return
        self._publish_manual_twist()

    def _close_for_shutdown(self) -> None:
        """Cierra la ventana cuando ROS se apaga (termina app.exec())."""
        app = QApplication.instance()
        if app is not None:
            app.quit()

    def _publish_manual_twist(self) -> None:
        """Publica /cmd_vel a partir del control FRAME (solo si está activo).

        El control NO genera velocidades articulares: emite un Twist
        cartesiano (normalizado con ganancia) que consume el servo.
        """
        if not self._manual_active or self._cmd_vel_pub is None:
            return
        if self._frame_panel is None:
            return
        comp = self._frame_panel.twist_components()
        msg = Twist()
        msg.linear.x = float(comp["linear"]["x"])
        msg.linear.y = float(comp["linear"]["y"])
        msg.linear.z = float(comp["linear"]["z"])
        msg.angular.x = float(comp["angular"]["x"])
        msg.angular.y = float(comp["angular"]["y"])
        msg.angular.z = float(comp["angular"]["z"])
        self._cmd_vel_pub.publish(msg)

    def _on_joint_states(self, msg: JointState) -> None:
        """Actualiza J1-J6 con la posición real de cada articulación."""
        positions = dict(zip(msg.name, msg.position))
        self._joints_now = positions
        for ros_name, gui_key in _ROS_JOINT_MAP.items():
            if ros_name in positions:
                self._joint_labels[gui_key].setText(f"{positions[ros_name]:.4f}")

    def _on_camera_image(self, msg: Image) -> None:
        """Convierte sensor_msgs/Image ('rgb8') a QImage y actualiza el visor.

        El callback se ejecuta en el hilo de la GUI (spin_once desde el
        QTimer), por lo que tocar widgets Qt aquí es seguro.
        """
        if self._camera_view is None:
            return
        if msg.encoding != _CAMERA_RGB8:
            if msg.encoding != self._warned_encoding:
                self._warned_encoding = msg.encoding
                self._log(
                    "warning",
                    f"Cámara: encoding '{msg.encoding}' no soportado (solo 'rgb8')",
                )
                self._ros_node.get_logger().warning(
                    f"/camera/image_raw usa '{msg.encoding}'; se espera "
                    f"'{_CAMERA_RGB8}'. Visor no actualizado."
                )
            return
        try:
            image = QImage(
                bytearray(msg.data),
                msg.width,
                msg.height,
                msg.step,
                QImage.Format_RGB888,
            ).copy()  # copia propia (desligada del buffer ROS del mensaje)
        except Exception as exc:  # noqa: BLE001 - no romper la GUI
            self._ros_node.get_logger().error(
                f"Fallo al convertir /camera/image_raw: {exc}"
            )
            return
        self._camera_view.set_frame(image)

    # ── ROS 2: estado de los controladores (modo exclusivo) ─────────────
    def _poll_controller_status(self) -> None:
        """Lanza una consulta asíncrona a /controller_manager/list_controllers."""
        if self._ctrl is None or self._switch_busy or not rclpy.ok():
            return
        self._ctrl.refresh(done_callback=self._on_controller_status_response)

    def _on_controller_status_response(self, states, ok, error=None) -> None:
        """Actualiza el indicador de estado y el modo actual de la GUI."""
        if not ok:
            return
        mode = derive_mode(states)
        self._update_mode_display(mode, states)
        if mode == "conflicto":
            # JTC y FVC activos: estado prohibido. Se vuelve a trayectoria.
            self._log(
                "warning",
                "CONFLICTO: joint_trajectory_controller y "
                "forward_velocity_controller activos; volviendo a TRAYECTORIA",
            )
            self._switch_to_mode("trayectoria", reason="resolver conflicto")

    def _update_mode_display(self, mode: str, states: dict | None = None) -> None:
        """Refleja el modo actual en el indicador MODO ACTUAL y en el estado."""
        if mode == "trayectoria":
            ctrl = JOINT_TRAJECTORY_CONTROLLER
        elif mode == "manual":
            ctrl = FORWARD_VELOCITY_CONTROLLER
        else:
            ctrl = None
        pack_mode = mode if mode in (
            "trayectoria", "manual", "seguro", "conflicto", "desconocido",
        ) else "desconocido"
        if self._frame_panel is not None:
            self._frame_panel.set_mode_state(
                pack_mode,
                controller=ctrl,
                velocity_pct=self._frame_panel.velocity_percent(),
            )
        colores = {
            "trayectoria": _VERDE,
            "manual": "#f39c12",
            "seguro": _GRIS_MED,
            "conflicto": _ROJO,
            "desconocido": _GRIS_MED,
        }
        textos = {
            "trayectoria": "● HABILITADO (MODO TRAYECTORIA)",
            "manual": "● MODO MANUAL (CONTROL FRAME)",
            "seguro": "● MODO SEGURO (sin controlador activo)",
            "conflicto": "● CONFLICTO DE CONTROLADORES",
            "desconocido": "● CONECTANDO...",
        }
        self._set_status(textos[mode], colores[mode])

    def closeEvent(self, event):  # noqa: N802 - firma de Qt
        """Detiene los timers ROS y evita callbacks tras cerrar la ventana."""
        if self._motion_active and self._motion_goal_handle is not None:
            try:
                self._motion_goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001 - no romper el cierre
                pass
        if getattr(self, "_manual_active", False):
            self._send_zero_twist()  # seguridad: robot quieto al cerrar
            self._manual_active = False
        if self._ros_timer is not None:
            self._ros_timer.stop()
            self._ros_timer = None
        if self._status_timer is not None:
            self._status_timer.stop()
            self._status_timer = None
        super().closeEvent(event)

    # ── Placeholders (sin ROS) ────────────────────────────────────────
    def _set_status(self, text: str, color: str):
        self._status_label.setText(text)
        self._status_label.setStyleSheet(_label_style(color, _FAMILIA, 15))

    # ── Cambio de controladores (modo excluyente TRAYECTORIA/MANUAL) ──
    def _switch_to_mode(
        self,
        mode: str,
        *,
        after=None,
        reason: str = "",
    ) -> None:
        """Solicita de forma asíncrona alcanzar `mode` cambiando controladores.

        Solo hay un cambio en curso a la vez (_switch_busy). Al terminar se
        llama a `after(ok, message)`. Si no hace falta cambiar nada se llama
        a `after` con ok=True.
        """
        if self._ctrl is None:
            if after is not None:
                after(False, "controller_manager no disponible")
            return
        if self._switch_busy:
            self._log(
                "warning",
                "CAMBIO DE CONTROLADOR EN CURSO: operación ignorada",
            )
            if after is not None:
                after(False, "ya hay un cambio de controlador en curso")
            return
        plan = switch_plan(mode)
        if not plan["activate"] and not plan["deactivate"]:
            if after is not None:
                after(True, "sin cambios necesarios")
            return
        self._after_switch = after
        self._switch_busy = True
        self.btn_enable.setEnabled(False)
        self.btn_disable.setEnabled(False)
        self._pause_status_poll()
        texto = mode.upper()
        self._log("info", f"{texto}: cambiando controladores ({reason})...")
        self._set_status(f"● {texto}: CAMBIANDO CONTROLADOR...", _AZUL)
        self._ctrl.switch(
            activate=plan["activate"],
            deactivate=plan["deactivate"],
            done_callback=lambda ok, states, msg: self._on_mode_switch_done(
                mode, ok, states, msg
            ),
        )

    def _on_mode_switch_done(self, mode: str, ok: bool, states, message: str) -> None:
        """Fin del cambio de controladores: actualiza GUI y ejecuta `after`."""
        self._switch_busy = False
        self.btn_enable.setEnabled(True)
        self.btn_disable.setEnabled(True)
        self._update_mode_display(derive_mode(states), states)
        self._resume_status_poll()
        if ok:
            self._log("ok", f"{mode.upper()}: controladores cambiados ({message})")
        else:
            self._log(
                "error",
                f"{mode.upper()}: FALLÓ el cambio de controladores ({message})",
            )
            self._set_status(
                f"● ERROR: no se pudo pasar a {mode.upper()} ({message})", _ROJO
            )
        after = self._after_switch
        self._after_switch = None
        if after is not None:
            after(ok, message)

    # ── Control manual (FRAME): ACTIVAR/VOLVER y /cmd_vel ───────────────
    def _on_manual_toggle(self, active: bool) -> None:
        """Activa/desactiva el control manual CAMBIANDO de controlador.

        Activar -> forward_velocity_controller (modo MANUAL); volver ->
        joint_trajectory_controller (modo TRAYECTORIA). El cambio es
        asíncrono y verificado; solo se publica /cmd_vel cuando se confirma.
        Si no hay ROS (placeholder) no se publica nada.
        """
        if self._ros_node is None or self._cmd_vel_pub is None:
            self._log(
                "info",
                f"CONTROL MANUAL {'ACTIVAR' if active else 'VOLVER'}"
                " (sin ROS: no publicado)",
            )
            return
        if self._switch_busy:
            self._log("warning", "CONTROL MANUAL: cambio de controlador en curso, espera")
            return
        objetivo = "manual" if active else "trayectoria"
        self._log(
            "info",
            f"{'ACTIVAR MANUAL' if active else 'VOLVER A TRAYECTORIA'}:"
            " solicitando cambio de controlador",
        )
        self._set_status(
            ("● ACTIVANDO CONTROL MANUAL..." if active else "● VOLVIENDO A TRAYECTORIA..."),
            _AZUL,
        )
        if self._frame_panel is not None:
            self._frame_panel.set_toggle_busy()  # bloqueado hasta confirmar

        def _despues(ok: bool, message: str) -> None:
            if not ok:
                # El switch falló: nada de publicar, estado seguro.
                self._manual_active = False
                self._send_zero_twist()
                if self._frame_panel is not None:
                    self._frame_panel.reset()
                self._set_status(
                    f"● ERROR: cambio a {objetivo} FALLÓ ({message})", _ROJO
                )
            else:
                self._manual_active = bool(active)
                if self._manual_active:
                    self._log("info", "CONTROL MANUAL ACTIVADO: publicando /cmd_vel")
                    self._set_status(
                        "● CONTROL MANUAL ACTIVADO (publicando /cmd_vel)", "#f39c12"
                    )
                else:
                    self._send_zero_twist()
                    if self._frame_panel is not None:
                        self._frame_panel.reset()
                    self._log("warning", "CONTROL MANUAL DESACTIVADO: Twist=0 enviado")
                    self._set_status(
                        "● CONTROL MANUAL DESACTIVADO (robot quieto)", _GRIS_CLR
                    )
            if self._frame_panel is not None:
                self._frame_panel.set_toggle_state(self._manual_active)

        self._switch_to_mode(objetivo, after=_despues, reason="manual")

    def _send_zero_twist(self) -> None:
        """Envía explícitamente un comando Twist=0 (seguridad)."""
        if self._cmd_vel_pub is not None:
            self._cmd_vel_pub.publish(Twist())
            if self._ros_node is not None:
                self._ros_node.get_logger().info(
                    "CONTROL MANUAL: Twist=0 publicado explícitamente."
                )

    # ── ENABLE / DISABLE (cambio de controladores asíncrono) ─────────
    def _on_enable(self):
        self._switch_to_mode_manual("trayectoria", "ENABLE")

    def _on_disable(self):
        self._switch_to_mode_manual("seguro", "DISABLE")

    def _switch_to_mode_manual(self, mode: str, accion: str) -> None:
        """ENABLE/DISABLE: no se cambia de controlador con un movimiento vivo."""
        if self._motion_active:
            self._log("warning", f"{accion}: movimiento en curso, ignorado")
            self._set_status("● CAMBIO DE MODO BLOQUEADO (movimiento en curso)", _AMARILLO)
            return
        self._switch_to_mode(mode, reason=f"{accion} manual")

    # ── Movimiento HOME + P1-P3 (Goal único, asíncrono, común) ─────────
    def _on_home(self):
        """Envía el robot a la posición cero (HOME) sin bloquear la GUI."""
        if self._motion_active:
            self._log("info", "HOME: movimiento en curso, acción ignorada")
            return
        if (
            self._ros_node is None
            or self._action_client is None
            or self._ctrl is None
        ):
            print("[GUI] HOME presionado (sin ROS: placeholder)")
            return

        goal = self._build_single_goal(_HOME_POSITIONS, _HOME_DURATION_S)
        self._run_goal(
            goal=goal,
            button=self.btn_home,
            moving_text="MOVIENDO A HOME...",
            ok_text="HOME COMPLETADO",
            tag="HOME",
        )

    def _on_punto(self, name: str):
        """Envía un único waypoint P1/P2/P3 (IK ya resuelta en el arranque)."""
        if self._motion_active:
            self._log("info", f"{name}: movimiento en curso, acción ignorada")
            return
        if self._ros_node is None or self._action_client is None:
            print(f"[GUI] {name} presionado (sin ROS: placeholder)")
            return
        if not self._point_ready:
            self._log("error", f"{name}: viajar no disponible (carga de plan fallida)")
            self._set_status(
                f"● {name}: NO DISPONIBLE (carga de trayectoria fallida)", _ROJO
            )
            return

        wp_name = _POINT_WAYPOINT_NAMES.get(name)
        btn = getattr(self, _POINT_BTN_ATTR.get(name, ""), None)
        if wp_name is None or btn is None:
            return
        q = self._point_qs.get(wp_name)
        if q is None:
            self._set_status(
                f"● {name}: waypoint '{wp_name}' NO ENCONTRADO en points.yaml", _ROJO
            )
            self._ros_node.get_logger().error(
                f"{name}: waypoint '{wp_name}' no existe en points.yaml."
            )
            return

        goal = self._build_single_goal(q, _POINT_DURATION_S)
        self._run_goal(
            goal=goal,
            button=btn,
            moving_text=f"MOVIENDO A {name}...",
            ok_text=f"{name} ALCANZADO",
            tag=name,
        )

    # ── Puntos del usuario (crear/editar/eliminar/mover) ──────────────
    def _load_user_points(self) -> None:
        """Carga el almacén de puntos del usuario (independiente de ROS)."""
        self._store = UserPointStore()
        self._store.load()
        if self._store.errors:
            for err in self._store.errors:
                self._log("warning", f"[PUNTOS] {err}")
        if self._store.names():
            self._log("info", f"[PUNTOS] {len(self._store.names())} punto(s) cargado(s)")
        self._refresh_points_list()

    def _save_user_points(self) -> None:
        try:
            self._store.save()
        except OSError as exc:  # noqa: BLE001 - no romper la GUI
            self._log("error", f"[PUNTOS] No se pudo guardar ({exc})")

    def _on_capture_position(self) -> None:
        """GUARDAR POSICIÓN ACTUAL: crea un UserPoint desde los J1-J6 reales.

        La q leída de /joint_states ES la solución exacta (no se llama a la
        IK inversa): FK con Tx260lKinematics y guardado en el mismo almacén
        de Puntos del usuario. El punto aparece de inmediato en la lista y
        queda seleccionado; NO se añade todavía a la trayectoria.
        """
        if self._store is None:
            return
        if self._motion_active:
            return
        if self._joints_now is None:
            self._set_status("● GUARDAR: no hay /joint_states todavía", _ROJO)
            return
        try:
            q = [self._joints_now[j] for j in _HOME_JOINT_NAMES]
        except KeyError as exc:
            self._set_status(f"● GUARDAR: /joint_states sin {exc}", _ROJO)
            return
        if self._user_kinematics is None:
            from staubli_trajectory.tx2_60l_ik import Tx260lKinematics

            self._user_kinematics = Tx260lKinematics()
        point = user_point_from_joint_position(
            self._default_next_point_name(), q, self._user_kinematics
        )
        if not point.inside_limits:
            self._set_status("● GUARDAR: posición fuera de límites", _ROJO)
            return
        self._store.add(point)
        self._save_user_points()
        self._refresh_points_list()
        self._points_list.setCurrentRow(list(self._store.names()).index(point.name))
        self._log("ok", f"[PUNTOS] Posición actual guardada como '{point.name}'")
        self._set_status(f"● POSICIÓN GUARDADA como {point.name}", _VERDE)

    def _refresh_points_list(self) -> None:
        if self._points_list is None:
            return
        self._points_list.blockSignals(True)
        self._points_list.clear()
        for name in self._store.names():
            self._points_list.addItem(name)
        self._points_list.blockSignals(False)
        self._update_point_buttons()

    def _current_point(self) -> UserPoint | None:
        if self._points_list is None or self._store is None:
            return None
        item = self._points_list.currentItem()
        if item is None:
            return None
        return self._store.get(item.text())

    def _update_point_buttons(self, *_args) -> None:
        has = self._current_point() is not None
        self.btn_edit_point.setEnabled(has)
        self.btn_delete_point.setEnabled(has)
        self.btn_move_point.setEnabled(has and not self._motion_active)
        self.btn_add_to_traj.setEnabled(has and not self._motion_active)
        self._update_trajectory_buttons()

    # ── Trayectoria definida por el usuario (orden y ejecución) ────────
    def _refresh_trajectory_list(self, select_row: int | None = None) -> None:
        """Refresca la lista de trayectoria a partir de self._user_trajectory."""
        if self._trajectory_list is None:
            return
        self._trajectory_list.blockSignals(True)
        self._trajectory_list.clear()
        for i, name in enumerate(self._user_trajectory, start=1):
            self._trajectory_list.addItem(f"{i}. {name}")
        if select_row is not None and 0 <= select_row < len(self._user_trajectory):
            self._trajectory_list.setCurrentRow(select_row)
        self._trajectory_list.blockSignals(False)
        self._update_trajectory_buttons()

    def _update_trajectory_buttons(self, *_args) -> None:
        """Habilita/deshabilita los controles de la trayectoria (y btn_run)."""
        if getattr(self, "_trajectory_list", None) is None:
            return
        busy = self._motion_active
        n = len(self._user_trajectory)
        row = self._trajectory_list.currentRow()
        self.btn_traj_up.setEnabled(not busy and 1 <= row < n)
        self.btn_traj_down.setEnabled(not busy and 0 <= row < n - 1)
        self.btn_traj_remove.setEnabled(not busy and 0 <= row < n)
        self.btn_traj_clear.setEnabled(not busy and n > 0)
        self.btn_run.setEnabled(not busy)
        self.btn_add_to_traj.setEnabled(
            not busy and self._current_point() is not None
        )

    def _on_add_to_trajectory(self) -> None:
        """Añade el punto del usuario seleccionado al final de la trayectoria."""
        if self._motion_active:
            return
        point = self._current_point()
        if point is None:
            self._log("warning", "TRAYECTORIA: selecciona un punto del usuario")
            return
        self._user_trajectory.append(point.name)
        self._refresh_trajectory_list(select_row=len(self._user_trajectory) - 1)
        self._log("ok", f"TRAYECTORIA: añadido {point.name}")

    def _on_traj_move_up(self) -> None:
        """Mueve el elemento seleccionado una posición hacia arriba."""
        self._move_trajectory_item(-1)

    def _on_traj_move_down(self) -> None:
        """Mueve el elemento seleccionado una posición hacia abajo."""
        self._move_trajectory_item(+1)

    def _move_trajectory_item(self, delta: int) -> None:
        if self._motion_active:
            return
        row = self._trajectory_list.currentRow()
        new = row + delta
        n = len(self._user_trajectory)
        if not (0 <= row < n and 0 <= new < n):
            return
        self._user_trajectory[row], self._user_trajectory[new] = (
            self._user_trajectory[new],
            self._user_trajectory[row],
        )
        self._refresh_trajectory_list(select_row=new)
        self._log("info", "TRAYECTORIA: orden actualizado")

    def _on_traj_remove(self) -> None:
        """Quita el elemento seleccionado de la trayectoria (sin borrar el punto)."""
        if self._motion_active:
            return
        row = self._trajectory_list.currentRow()
        n = len(self._user_trajectory)
        if not (0 <= row < n):
            return
        name = self._user_trajectory.pop(row)
        select = min(row, len(self._user_trajectory) - 1)
        self._refresh_trajectory_list(select_row=select if select >= 0 else None)
        self._log("info", f"TRAYECTORIA: eliminado {name}")

    def _on_traj_clear(self) -> None:
        """Vacía únicamente la trayectoria actual (los puntos se conservan)."""
        if self._motion_active:
            return
        if not self._user_trajectory:
            return
        self._user_trajectory.clear()
        self._refresh_trajectory_list()
        self._log("info", "TRAYECTORIA: lista limpiada")

    def _resolve_user_trajectory(self) -> list[UserPoint] | None:
        """Valida los nombres del usuario; devuelve los UserPoint en orden o None."""
        resolved: list[UserPoint] = []
        for name in self._user_trajectory:
            point = self._store.get(name) if self._store is not None else None
            if point is None:
                self._log("error", f"TRAYECTORIA: '{name}' ya no existe")
                return None
            if (
                not point.solved
                or not point.inside_limits
                or point.q is None
                or len(point.q) != 6
            ):
                self._log("error", f"TRAYECTORIA: '{name}' sin q válida")
                return None
            resolved.append(point)
        return resolved

    def _default_next_point_name(self) -> str:
        n = 1
        while self._store.has(f"Punto {n}"):
            n += 1
        return f"Punto {n}"

    def _build_point_dialog(
        self,
        *,
        title: str,
        original_name: str | None = None,
        initial: UserPoint | None = None,
        default_name: str | None = None,
    ):
        """Diálogo modal de edición/creación de un punto (tool0, grados)."""
        if initial is not None:
            default_name = initial.name
        dlg = QDialog(self)
        dlg.setWindowTitle(f"{title} — Puntos del usuario")
        dlg.setModal(True)
        dlg.setMinimumWidth(380)
        dlg.setStyleSheet(
            f"QDialog {{ background-color: {_NEGRO}; }}"
            f"QLabel {{ color: {_BLANCO_HUESO}; }}"
            f"QLineEdit {{ background-color: {_GRIS_OSC}; color: {_BLANCO};"
            f" border: 1px solid {_GRIS_MED}; border-radius: 4px;"
            f" padding: 4px 8px; font-family: {_FAM_MONO}; }}"
        )
        root = QVBoxLayout(dlg)
        root.setSpacing(10)

        titulo = QLabel(title)
        titulo.setStyleSheet(
            f"font-size: 16px; font-weight: bold; color: {_AMARILLO};"
        )
        root.addWidget(titulo)

        keys = ["Nombre", "X", "Y", "Z", "RX", "RY", "RZ"]
        etiquetas = [
            "Nombre",
            "X (m)",
            "Y (m)",
            "Z (m)",
            "RX (grados)",
            "RY (grados)",
            "RZ (grados)",
        ]
        grid = QGridLayout()
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(8)
        fields: dict = {}
        for i, (key, etiqueta) in enumerate(zip(keys, etiquetas)):
            lbl = QLabel(etiqueta)
            grid.addWidget(lbl, i, 0, Qt.AlignRight)
            edit = QLineEdit()
            edit.setPlaceholderText(
                key if key == "Nombre" else ("0.300" if key in "XYZ" else "0.0")
            )
            grid.addWidget(edit, i, 1)
            fields[key] = edit
        root.addLayout(grid)

        error_label = QLabel("")
        error_label.setStyleSheet(
            f"color: {_ROJO_CLARO}; font-size: 12px; font-weight: bold;"
        )
        error_label.setWordWrap(True)
        error_label.hide()
        root.addWidget(error_label)

        def _guardar() -> None:
            parsed = self._validate_point_form(
                fields, error_label, original_name=original_name
            )
            if parsed is None:
                return
            name, position, rpy = parsed
            point, reason = self._resolve_point(name, position, rpy)
            if point is None:
                error_label.setText(reason)
                error_label.show()
                self._log("error", f"{name} NO GUARDADO: {reason}")
                return
            dlg._resolved_point = point
            dlg.accept()

        if initial is not None:
            fields["Nombre"].setText(initial.name)
            for key, valor in zip(("X", "Y", "Z"), initial.position_xyz):
                fields[key].setText(f"{valor:.4f}")
            for key, valor in zip(("RX", "RY", "RZ"), initial.rpy_degrees):
                fields[key].setText(f"{valor:.4f}")
        elif default_name:
            fields["Nombre"].setText(default_name)

        btns = QHBoxLayout()
        btn_save = _make_btn("GUARDAR PUNTO", _btn_style(_VERDE, min_w=140, min_h=34))
        btn_cancel = _make_btn("CANCELAR", _btn_style(_GRIS_MED, min_w=120, min_h=34))
        btn_save.clicked.connect(_guardar)
        btn_cancel.clicked.connect(dlg.reject)
        btns.addStretch()
        btns.addWidget(btn_save)
        btns.addWidget(btn_cancel)
        root.addLayout(btns)

        return dlg

    def _validate_point_form(
        self, fields: dict, error_label: QLabel, original_name: str | None = None
    ):
        """Valida los campos del diálogo; devuelve (nombre, pos, rpy) o None."""
        nombre = fields["Nombre"].text().strip()
        if not nombre:
            error_label.setText("El nombre no puede estar vacío.")
            error_label.show()
            return None
        if self._store.has(nombre) and nombre != original_name:
            error_label.setText(f"Ya existe un punto llamado '{nombre}'.")
            error_label.show()
            return None

        valores: list[float] = []
        for key, etiqueta in zip(
            ("X", "Y", "Z", "RX", "RY", "RZ"),
            ("X", "Y", "Z", "RX", "RY", "RZ"),
        ):
            texto = fields[key].text().strip()
            if not texto:
                error_label.setText(f"El campo {etiqueta} está vacío.")
                error_label.show()
                return None
            try:
                valores.append(float(texto))
            except ValueError:
                error_label.setText(f"'{texto}' no es un número válido en {etiqueta}.")
                error_label.show()
                return None
        return nombre, valores[0:3], valores[3:6]

    def _resolve_point(self, name: str, position_xyz: list, rpy_degrees: list):
        """IK de tool0 -> base_link reutilizando Tx260lKinematics.

        Devuelve (UserPoint, '') si la IK resuelve dentro de límites, o
        (None, motivo) si no; en ese caso el punto NO se guarda.
        """
        quaternion = rpy_degrees_to_quaternion_xyzw(*rpy_degrees)
        if self._user_kinematics is None:
            from staubli_trajectory.tx2_60l_ik import Tx260lKinematics

            self._user_kinematics = Tx260lKinematics()

        q_ref = None
        if self._joints_now is not None:
            try:
                q_ref = [self._joints_now[j] for j in _HOME_JOINT_NAMES]
            except KeyError:
                q_ref = None

        frame = self._user_kinematics.frame_from_position_quaternion(
            position_xyz, quaternion
        )
        res = self._user_kinematics.inverse_kinematics(
            frame, q_prev=q_ref, q_state=q_ref
        )
        if not res.solved:
            return None, "IK sin solución para esas coordenadas."
        if not res.inside_limits:
            fuera = [
                joint
                for joint, ok in zip(_HOME_JOINT_NAMES, res.joints_inside)
                if not ok
            ]
            return None, f"FUERA DE LÍMITES: {', '.join(fuera)}."
        return (
            UserPoint(
                name=name,
                position_xyz=[float(v) for v in position_xyz],
                rpy_degrees=[float(v) for v in rpy_degrees],
                quaternion_xyzw=[float(v) for v in quaternion],
                q=[float(v) for v in res.q],
                solved=True,
                inside_limits=True,
            ),
            "",
        )

    def _new_point(self) -> None:
        if self._store is None:
            return
        dlg = self._build_point_dialog(
            title="NUEVO PUNTO",
            original_name=None,
            default_name=self._default_next_point_name(),
        )
        if dlg.exec() == QDialog.Accepted:
            point = getattr(dlg, "_resolved_point", None)
            if point is None:
                return
            self._log("ok", f"[PUNTOS] IK válida para '{point.name}'")
            self._store.add(point)
            self._save_user_points()
            self._refresh_points_list()
            self._log("ok", f"[PUNTOS] Punto '{point.name}' creado")

    def _edit_point(self) -> None:
        point = self._current_point()
        if point is None or self._store is None:
            return
        dlg = self._build_point_dialog(
            title=f"EDITAR {point.name}",
            original_name=point.name,
            initial=point,
        )
        if dlg.exec() == QDialog.Accepted:
            nuevo = getattr(dlg, "_resolved_point", None)
            if nuevo is None:
                return
            self._log("ok", f"[PUNTOS] IK válida para '{nuevo.name}'")
            self._store.update(nuevo, previous_name=point.name)
            self._save_user_points()
            self._refresh_points_list()
            self._log("ok", f"[PUNTOS] Punto '{point.name}' actualizado")

    def _delete_point(self) -> None:
        point = self._current_point()
        if point is None or self._store is None:
            return
        self._store.remove(point.name)
        self._save_user_points()
        self._refresh_points_list()
        self._log("ok", f"[PUNTOS] Punto '{point.name}' eliminado")

    def _move_to_point(self) -> None:
        point = self._current_point()
        if point is None or self._store is None:
            return
        goal = self._build_single_goal(point.q, _POINT_DURATION_S)
        self._run_goal(
            goal=goal,
            button=self.btn_move_point,
            moving_text=f"MOVIENDO A {point.name}...",
            ok_text=f"{point.name} ALCANZADO",
            tag=point.name,
            fail_text=f"{point.name}: MOVIMIENTO FALLIDO",
        )

    def _run_goal(
        self,
        *,
        goal: FollowJointTrajectory.Goal,
        button,
        moving_text: str,
        ok_text: str,
        tag: str,
        fail_text: str | None = None,
    ) -> None:
        """Envío común HOME/P1-P3/TRAYECTORIA/PUNTO: asegura controlador y sigue.

        Antes de enviar el goal garantiza estar en modo TRAYECTORIA
        (joint_trajectory_controller activo, forward_velocity_controller
        desactivado) cambiando controladores si hiciera falta y, solo cuando
        el cambio se confirma, lanza send_goal_async. 'fail_text', si se da,
        personaliza el mensaje/estado de fallo.
        """
        if self._motion_active:
            return
        if self._ros_node is None or self._action_client is None or self._ctrl is None:
            print(f"[GUI] {tag} sin ROS (placeholder)")
            return

        self._motion_active = True
        self._motion_button = button
        button.setEnabled(False)  # evita goals simultáneos
        self._pause_status_poll()
        self._log("info", f"{tag}: solicitando movimiento...")
        self._set_status(f"● {tag}: PREPARANDO CONTROLADOR...", _AZUL)

        def _tras_switch(ok: bool, message: str) -> None:
            if self._stop_requested:
                self._log("warning", f"{tag}: STOP antes de enviar el goal")
                self._finish_motion(
                    "● MOVIMIENTO DETENIDO (goal no enviado)", _AMARILLO
                )
                return
            if not ok:
                self._log(
                    "error",
                    f"{tag}: joint_trajectory_controller no activo ({message})",
                )
                self._finish_motion(
                    fail_text
                    and f"● {fail_text} (controller no activo)"
                    or f"● {tag}: CONTROLLER NO ACTIVO ({message})",
                    _ROJO,
                )
                return
            if not self._action_client.server_is_ready():
                self._finish_motion(f"● {tag}: ACTION SERVER NO DISPONIBLE", _ROJO)
                return
            self._set_status(moving_text, _AZUL)
            self._ros_node.get_logger().info(
                f"{tag}: enviando goal a "
                f"{[float(v) for v in goal.trajectory.points[0].positions]}."
            )
            goal_future = self._action_client.send_goal_async(goal)
            goal_future.add_done_callback(
                lambda fut: self._on_motion_goal_response(
                    fut, button=button, ok_text=ok_text, tag=tag, fail_text=fail_text
                )
            )

        self._switch_to_mode("trayectoria", after=_tras_switch, reason=f"goal {tag}")

    # ── Construcción del goal y seguimiento asíncrono ─────────────────
    def _build_single_goal(
        self, positions, duration_s: float
    ) -> FollowJointTrajectory.Goal:
        """Goal de un solo waypoint: trayectoria + tolerancias (como antes)."""
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.time_from_start = Duration(seconds=duration_s).to_msg()

        trajectory = JointTrajectory()
        trajectory.joint_names = list(_HOME_JOINT_NAMES)
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        self._fill_goal_tolerances(goal)
        return goal

    def _build_trajectory_goal(self) -> FollowJointTrajectory.Goal:
        """Goal multi-waypoint reutilizando plan.trajectory (sin rehacer IK)."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._point_plan.trajectory
        self._fill_goal_tolerances(goal)
        return goal

    def _build_user_trajectory_goal(self) -> FollowJointTrajectory.Goal:
        """Goal multi-waypoint desde la lista del usuario (q ya resuelta).

        Un JointTrajectoryPoint por punto, positions = point.q (rad) y
        time_from_start = 5/10/15... s (paso _USER_TRAJ_STEP_S).
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = list(_HOME_JOINT_NAMES)
        for i, name in enumerate(self._user_trajectory, start=1):
            point = self._store.get(name) if self._store is not None else None
            if point is None or point.q is None:
                continue
            msg_point = JointTrajectoryPoint()
            msg_point.positions = [float(v) for v in point.q]
            msg_point.time_from_start = Duration(
                seconds=_USER_TRAJ_STEP_S * i
            ).to_msg()
            trajectory.points.append(msg_point)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        self._fill_goal_tolerances(goal)
        return goal

    def _fill_goal_tolerances(self, goal: FollowJointTrajectory.Goal) -> None:
        """Aplica las tolerancias del goal (mismas que staubli_trajectory)."""
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

    def _on_motion_goal_response(
        self, future, *, button, ok_text: str, tag: str, fail_text: str | None = None
    ):
        """El controller acepta/rechaza el goal; espera el resultado real."""
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001 - no romper la GUI
            self._finish_motion(f"● {tag}: ERROR AL ENVIAR ({exc})", _ROJO)
            return
        if not goal_handle.accepted:
            self._log("error", f"{tag}: goal RECHAZADO")
            self._finish_motion(
                fail_text and f"● {fail_text}" or f"● {tag}: GOAL RECHAZADO", _ROJO
            )
            return

        self._motion_goal_handle = goal_handle
        self._log("info", f"{tag}: goal ACEPTADO")
        self._ros_node.get_logger().info(
            f"{tag}: goal ACEPTADO; esperando resultado de ejecución."
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut: self._on_motion_result(
                fut, ok_text=ok_text, tag=tag, fail_text=fail_text
            )
        )

    def _on_motion_result(
        self, future, *, ok_text: str, tag: str, fail_text: str | None = None
    ) -> None:
        """Comprueba el estado final del goal (no basta con haberlo enviado).

        Nota: joint_trajectory_controller informa SUCCESSFUL al cancelar; por
        eso se comprueba primero _stop_requested (STOP/cancelación).
        """
        if not self._motion_active:
            return  # resultado de un goal ya cerrado (doble callback)
        try:
            wrapped = future.result()
            error_code = getattr(wrapped.result, "error_code", None)
            error_string = getattr(wrapped.result, "error_string", None) or ""
        except Exception as exc:  # noqa: BLE001 - no romper la GUI
            self._log("error", f"{tag}: error al leer el resultado ({exc})")
            self._finish_motion(f"● {tag}: ERROR ({exc})", _ROJO)
            return

        if self._stop_requested:
            self._log("warning", f"{tag}: goal cancelado por STOP")
            self._finish_motion("● MOVIMIENTO DETENIDO", _AMARILLO)
            return

        if error_code == _HOME_MSG.Result.SUCCESSFUL:
            self._log("ok", f"{tag}: {ok_text}")
            self._finish_motion(f"● {ok_text}", _VERDE)
            return

        nombre = _HOME_RESULT_CODES.get(error_code, f"desconocido ({error_code})")
        if (
            error_code == _HOME_MSG.Result.INVALID_GOAL
            and "deactivate" in error_string.lower()
        ):
            self._log(
                "warning",
                f"{tag}: detenido (joint_trajectory_controller desactivado)",
            )
            self._finish_motion(
                "● MOVIMIENTO DETENIDO (controller desactivado)", _AMARILLO
            )
            return
        detalle = f"{nombre}" + (f" — {error_string}" if error_string else "")
        self._log("error", f"{tag}: goal FALLÓ ({detalle})")
        self._finish_motion(
            fail_text and f"● {fail_text}" or f"● {tag} FALLÓ: {nombre}", _ROJO
        )

    def _finish_motion(self, status_text: str, color: str) -> None:
        """Restaura botón y sondeo de estado al terminar (o fallar) un goal."""
        self._motion_active = False
        self._motion_goal_handle = None
        self._stop_requested = False
        if self._motion_button is not None:
            self._motion_button.setEnabled(True)
            self._motion_button = None
        self._set_status(status_text, color)
        self._update_point_buttons()
        QTimer.singleShot(_HOME_STATUS_HOLD_MS, self._resume_status_poll)

    def _pause_status_poll(self) -> None:
        """Evita que el indicador (1 Hz) pise el estado del movimiento en curso."""
        if self._status_timer is not None:
            self._status_timer.stop()

    def _resume_status_poll(self) -> None:
        """Reanuda el indicador (salvo que haya otro movimiento en curso)."""
        if self._status_timer is not None and not self._motion_active:
            self._status_timer.start()
            self._poll_controller_status()

    # ── P1/P2/P3: carga única de points.yaml + IK (sin duplicar) ──────
    def _load_point_plan(self) -> None:
        """Resuelve las IK de points.yaml con staubli_trajectory.

        Si la carga o la IK fallan, la GUI NO se cierra: P1-P3 quedan
        deshabilitados y se muestra el error.
        """
        try:
            from pathlib import Path

            from ament_index_python.packages import get_package_share_directory
            from staubli_trajectory.points_config import PointsConfig
            from staubli_trajectory.trajectory_builder import TrajectoryBuilder

            share = Path(get_package_share_directory(_POINTS_PKG))
            config_path = share.joinpath(*_POINTS_RELATIVE_CONFIG)
            config = PointsConfig(str(config_path))
            plan = TrajectoryBuilder(config).build()
        except Exception as exc:  # noqa: BLE001 - no cerrar la GUI
            self._ros_node.get_logger().error(f"P1-P3 no disponibles: {exc}")
            self._log("error", f"Plan no disponible: {exc}")
            self._set_status("● P1-P3 NO DISPONIBLES (carga fallida)", _ROJO)
            self._set_points_enabled(False)
            return

        if not plan.all_solved or not plan.all_inside_limits:
            self._ros_node.get_logger().error(
                "P1-P3 no disponibles: IK sin solución o fuera de límites."
            )
            self._log("error", "Plan no disponible: IK sin solución o fuera de límites")
            self._set_status("● P1-P3 NO DISPONIBLES (IK no resuelta)", _ROJO)
            self._set_points_enabled(False)
            return

        qs: dict = {}
        for record in plan.records:
            if getattr(record, "name", None) is not None:
                qs[record.name] = [float(v) for v in record.q]
        self._point_plan = plan
        self._point_qs = qs
        self._point_ready = True
        self._set_points_enabled(True)
        self._log("ok", f"Plan cargado: {len(plan.records)} waypoints")
        self._ros_node.get_logger().info(
            "P1-P3 listos: waypoints resueltos desde points.yaml "
            f"({[r.name for r in plan.records]})."
        )

    def _set_points_enabled(self, enabled: bool) -> None:
        """Habilita/deshabilita P1-P3 (btn_run lo gestiona la trayectoria)."""
        for attr in _POINT_BTN_ATTR.values():
            btn = getattr(self, attr, None)
            if btn is not None:
                btn.setEnabled(enabled)

    # ── EJECUTAR TRAYECTORIA (trayectoria definida por el usuario) ───────
    def _on_run(self):
        """Envía la trayectoria de puntos de usuario como goal multi-waypoint."""
        if self._motion_active:
            self._log("info", "EJECUTAR TRAYECTORIA: movimiento en curso, ignorado")
            return
        if (
            self._ros_node is None
            or self._action_client is None
            or self._ctrl is None
        ):
            print("[GUI] EJECUTAR TRAYECTORIA presionado (sin ROS: placeholder)")
            return
        if not self._user_trajectory:
            self._log("warning", "TRAYECTORIA: no hay puntos definidos")
            self._set_status("● TRAYECTORIA: no hay puntos definidos", _AMARILLO)
            return

        resolved = self._resolve_user_trajectory()
        if resolved is None:
            return
        self._log("info", f"TRAYECTORIA: preparando {len(resolved)} puntos...")
        goal = self._build_user_trajectory_goal()
        self._log("info", "TRAYECTORIA: enviando goal...")
        self._run_goal(
            goal=goal,
            button=self.btn_run,
            moving_text="EJECUTANDO TRAYECTORIA...",
            ok_text="TRAYECTORIA COMPLETADA",
            tag="TRAYECTORIA",
            fail_text="TRAYECTORIA: MOVIMIENTO FALLIDO",
        )

    # ── STOP (control manual o trayectoria: robot quieto) ──────────────────
    def _on_stop(self):
        """Detiene el movimiento en curso.

        - Si el control manual está ACTIVO (o activándose): Twist=0 y cambio
          de controlador VOLVER A TRAYECTORIA (deja el JTC listo para HOME/P1-P3).
        - Si hay un goal de trayectoria activo: lo cancela.
        """
        if self._manual_active:
            self._log(
                "warning", "STOP: desactivando control manual y volviendo a TRAYECTORIA"
            )
            self._on_manual_toggle(False)
            return
        if not self._motion_active:
            self._set_status("● SIN MOVIMIENTO EN CURSO", _GRIS_CLR)
            self._log("info", "STOP: no hay movimiento en curso")
            return
        self._stop_requested = True
        if self._motion_goal_handle is None:
            # El goal aún no se envió (comprobando controller): aborta el envío.
            self._set_status("● DETENIENDO (antes de enviar)...", _AZUL)
            self._log("warning", "STOP: cancelando envío pendiente")
            return
        self._set_status("● DETENIENDO MOVIMIENTO...", _AZUL)
        self._log("warning", "STOP: cancelando goal activo")
        try:
            future = self._motion_goal_handle.cancel_goal_async()
            future.add_done_callback(self._on_cancel_response)
        except Exception as exc:  # noqa: BLE001 - no romper la GUI
            self._log("error", f"STOP: no se pudo cancelar ({exc})")
            self._stop_requested = False

    def _on_cancel_response(self, future) -> None:
        """Interpreta la respuesta del servicio de cancelación (action_msgs)."""
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - no romper la GUI
            self._log("error", f"STOP: error en cancelación ({exc})")
            return
        code = response.return_code
        if code == CancelGoal.Response.ERROR_NONE:
            self._log("info", "STOP: cancelación aceptada")
        elif code == CancelGoal.Response.ERROR_REJECTED:
            self._log("warning", "STOP: cancelación rechazada (¿ya terminó?)")
            self._stop_requested = False
        elif code == CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID:
            self._log("warning", "STOP: goal desconocido (¿ya terminó?)")
            self._stop_requested = False
        else:  # ERROR_GOAL_TERMINATED
            self._log("warning", "STOP: el goal ya había terminado")
            self._stop_requested = False

    # ── Panel de mensajes ──────────────────────────────────────────────
    def _log(self, level: str, text: str) -> None:
        """Añade una línea al panel de mensajes con color y timestamp."""
        editor = getattr(self, "_log_edit", None)
        if editor is None:
            return
        color = {
            "info": _GRIS_CLR,
            "ok": _VERDE,
            "warning": _AMARILLO,
            "error": _ROJO_CLARO,
        }.get(level, _GRIS_CLR)
        symbol = {"info": "·", "ok": "✓", "warning": "⚠", "error": "✗"}.get(level, "·")
        stamp = time.strftime("%H:%M:%S")
        cursor = editor.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)
        fmt = QTextCharFormat()
        fmt.setForeground(QColor(color))
        cursor.insertText(f"{stamp} {symbol} {text}\n", fmt)
        editor.setTextCursor(cursor)
        editor.ensureCursorVisible()

    def _clear_log(self) -> None:
        """Vacía el panel de mensajes."""
        self._log_edit.clear()
        self._log("info", "registro limpiado")


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
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001 - el contexto pudo cerrarse ya (SIGINT)
            pass
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()