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
from PySide6.QtCore import QPointF, QSize, Qt, QTimer
from PySide6.QtGui import (
    QColor,
    QFont,
    QImage,
    QPainter,
    QPen,
    QPixmap,
    QTextCharFormat,
    QTextCursor,
)
from PySide6.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QComboBox,
    QDialog,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLayout,
    QLineEdit,
    QListWidget,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
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
from .scan_path import (
    FACE_NORMALS,
    ROLES,
    PartBox,
    ScanPathSpec,
    ScanPathStore,
    ScanProfile,
    load_kinematics,
    resolve_scan,
)
from .scan_execution import (
    build_scan_execution,
    describe_plan,
    waypoints_to_dicts,
)
from .sensor_reference import (
    CAMERA_HFOV_RAD,
    DEFAULT_WORKING_DISTANCE_M,
    working_reference_pixels,
)
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

# ─── Panel de barrido: geometría derivada, SIN conexión al movimiento ─────
# Factor de conversión entre los milímetros que muestra/edita la GUI y los
# metros que usa scan_path.py. Es la ÚNICA conversión de unidades aquí: ni la
# geometría, ni la orientación, ni la IK se calculan en la GUI.
_SCAN_MM = 1000.0

# Dirección de barrido por defecto para cada cara, en base_link. Se eligen
# tangentes a la cara para que la línea nunca sea paralela a la normal (la
# geometría de scan_path proyecta el eje sobre el plano tangente y rechaza un
# eje degenerado). Al cambiar de cara la GUI propone este valor, que el
# usuario puede sobrescribir. No hay planificación raster.
#
# Para la cara +Z el sentido es +Y a propósito: el robot está en el lado -Y de
# la mesa, así que el barrido ARRANCA en el borde próximo (-Y) y termina en el
# lejano (+Y). P1 queda fuera de la pieza por el lado del robot, nunca encima
# de ella. Es solo el valor PROPUESTO al elegir cara: el sentido es del usuario
# y el cálculo lo respeta sea cual sea (no presupone +Y ni -Y).
_SCAN_DEFAULT_AXIS = {
    "+Z": (0.0, 1.0, 0.0),
    "-Z": (1.0, 0.0, 0.0),
    "+X": (0.0, 1.0, 0.0),
    "-X": (0.0, 1.0, 0.0),
    "+Y": (1.0, 0.0, 0.0),
    "-Y": (1.0, 0.0, 0.0),
}

# Rangos de los selectores de longitud. La distancia se mide desde la cámara
# hasta el PLANO DE APOYO (la mesa), no hasta la cara superior de la pieza.
# El valor inicial es la distancia óptima de trabajo (325 mm): el mismo
# parámetro configurable que la línea verde de referencia del sensor
# (sensor_reference.DEFAULT_WORKING_DISTANCE_M). El ajuste sigue siendo del
# usuario; la línea roja del cono FOV no cambia.
_SCAN_DIST_MIN_MM, _SCAN_DIST_MAX_MM, _SCAN_DIST_STEP_MM = 100, 500, 10
_SCAN_DIST_DEFAULT_MM = round(DEFAULT_WORKING_DISTANCE_M * _SCAN_MM)
_SCAN_LEN_MIN_MM, _SCAN_LEN_MAX_MM, _SCAN_LEN_STEP_MM = 10, 500, 5
# Márgenes ASIMÉTRICOS del barrido, independientes entre sí y medidos SIEMPRE a
# lo largo del sentido de barrido `line_axis` (nunca sobre una coordenada
# suelta): la línea sigue siendo perpendicular a la cara y `scan_distance`
# constante en todo el segmento.
#
#   scan_margin_start -> extiende P1 hacia el lado de ARRANQUE, ANTES de entrar
#                        en la pieza (borde de entrada desplazado hacia fuera).
#   scan_margin_end   -> extiende P2 hacia el lado de LLEGADA, DESPUÉS de salir
#                        de la pieza (borde opuesto desplazado hacia fuera).
#
# El código no presupone ninguna dirección: con el sentido por defecto de +Z
# ([0,1,0]) el arranque cae en -Y y la llegada en +Y, pero cualquier otro
# `line_axis` se recalcula igual (ver `ScanProfile.scan_span`).
#
# Los dos valores por defecto NO son el mismo número porque los extremos no
# tienen la misma holgura cinemática sobre joint_5 (soft limit −1.9944 rad): el
# de arranque es el más exigido (5 mm, prudente) y el de llegada el holgado
# (20 mm, de sobra). Ambos están medidos con la IK real y son ajustables.
_SCAN_MARGIN_MIN_MM, _SCAN_MARGIN_MAX_MM, _SCAN_MARGIN_STEP_MM = 0, 200, 5
_SCAN_MARGIN_START_DEFAULT_MM = 5
_SCAN_MARGIN_END_DEFAULT_MM = 20

# ── Barrido: APROXIMACIÓN a P1 (movimiento separado del escaneo) ────────
# El barrido se planifica como P1(t=0) -> P2: su primer punto da por
# hecho que el robot YA está en P1. Si se ejecuta desde otra configuración el
# joint_trajectory_controller recorta el primer comando (soft-limiter de
# gz_ros2_control) y el goal muere con GOAL_TOLERANCE_VIOLATED. Por eso la
# aproximación a P1 es un goal aparte, nunca parte del barrido.

# Umbral para considerar que el robot ya está en P1, en radianes y por
# articulación (norma infinita, la misma métrica por la que el JTC comprueba su
# goal_tolerance). 0.02 rad deja margen sobre los 0.01 rad de
# _HOME_GOAL_TOL_POS: llegado ese punto, el goal del barrido arranca dentro de
# su tolerancia de meta desde el primer ciclo.
_SCAN_AT_P1_TOL_RAD = 0.02

# IR A P1: UN SOLO waypoint, con un `time_from_start` calculado a partir de los
# límites reales de las articulaciones.
#
# El movimiento lo interpreta el JTC (spline propio desde el estado medido hasta
# P1): la GUI no manda ningún perfil generado. Lo único que hay que acertar es
# el TIEMPO, porque un `time_from_start` corto hace que JTC compruebe que la meta se
# ha agotado y aborte con GOAL_TOLERANCE_VIOLATED.
#
# El cálculo es el de un movimiento trapezoidal clásico, articulations:
#
#     t_acc = v / a
#     T     = 2·t_acc + max(0, |Δq| − v²/a) / v
#
# es decir, el tiempo mínimo necesario para recorrer |Δq| respetando una
# velocidad v y una aceleración a, y se toma el máximo sobre las seis
# articulaciones. No es un perfil que se envíe: solo dimensiona el reloj que
# le damos al controlador.
#
# v y a NO son los límites nominales, sino una fracción de ellos
# (_SCAN_GOTO_P1_VEL_FRACTION / _ACC_FRACTION). Razón medida: aunque el URDF
# declara 300 deg/s (5.236 rad/s) para joint_1..joint_5, en simulación joint_5
# solo llegó a ~0.73 rad/s; el límite real es el par (8 N·m), no el número
# del XML. Planificar al 100 % del límite declarado reproduciría el fallo
# original; al 12 % la velocidad pico de joint_4 queda en 0.63 rad/s, por
# debajo de lo que el robot demostró poder seguir sin saturarse.
#
# El factor de seguridad se aplica al tiempo total: JTC llega a la meta
# exactamente en t = time_from_start y, en simulación, siempre con un pequeño
# retardo de seguimiento. Sin él, cualquier milisegundo de desfase se paga con
# un ABORT en lugar de una llegada.
#
# El margen final lo da _SCAN_GOTO_P1_TIME_TOL_S en goal_time_tolerance, que es
# tiempo de ASENTAMIENTO, no una forma de esconder un tiempo mal calculado:
# si el tiempo de trayectoria fuese el correcto, el goal entraría antes de
# agotarlo.
_SCAN_GOTO_P1_VEL_FRACTION = 0.12
_SCAN_GOTO_P1_ACC_FRACTION = 0.12
_SCAN_GOTO_P1_TIME_SAFETY = 1.5
_SCAN_GOTO_P1_MIN_S = 1.5
_SCAN_GOTO_P1_MAX_S = 30.0
# Asentamiento tras la llegada, por encima de _HOME_GOAL_TIME_TOL_S (2.5 s),
# que es lo que se estaba agotando con 3.17 s de trayectoria.
_SCAN_GOTO_P1_TIME_TOL_S = 4.0

# Límites nominales reales de las articulaciones, en rad/s y rad/s², leídos de
# src/staubli_driver_ros2/staubli_robot_description/config/tx2_60l/joint_limits.yaml
# (300 deg/s y 1000 deg/s² en joint_1..joint_5; 500 deg/s y 3000 deg/s² en
# joint_6). Son el techo físico, no el ritmo al que se manda el goal.
_SCAN_GOTO_P1_MAX_VEL_RAD_S = (
    5.235987755982989,
    5.235987755982989,
    5.235987755982989,
    5.235987755982989,
    5.235987755982989,
    8.726646259971648,
)
_SCAN_GOTO_P1_MAX_ACC_RAD_S2 = (
    17.453292519943295,
    17.453292519943295,
    17.453292519943295,
    17.453292519943295,
    17.453292519943295,
    52.359877559829885,
)

# Soft limits (hard ± 0.1 rad) para TX2-60L, usados para corregir
# la duración de IR A P1 cuando alguna articulación está muy cerca del soft limit.
# No se cambian los soft limits de seguridad ni la validación.
_SCAN_GOTO_P1_SOFT_LOWER = (
    -3.141593 + 0.1,      # joint_1
    -2.216568 + 0.1,      # joint_2
    -2.617994 + 0.1,      # joint_3
    -4.712389 + 0.1,      # joint_4
    -2.094395 + 0.1,      # joint_5  → ~ -1.994395
    -4.712389 + 0.1,      # joint_6
)
_SCAN_GOTO_P1_SOFT_UPPER = (
    +3.141593 - 0.1,
    +2.216568 - 0.1,
    +2.617994 - 0.1,
    +4.712389 - 0.1,
    +2.094395 - 0.1,      # joint_5  → ~ +1.994395
    +4.712389 - 0.1,
)

# Columnas de la tabla de puntos derivados (P1/P2).
_SCAN_TABLE_HEADERS = (
    "Punto",
    "X (m)",
    "Y (m)",
    "Z (m)",
    "RPY (°)",
    "Dist. (mm)",
    "IK",
    "Límites",
    "Culpable",
)


# ── Unidades del panel de barrido (GUI en mm, scan_path en metros) ───────
def _spin_to_m(mm: int) -> float:
    """Milímetros del spin box → metros para scan_path."""
    return mm / _SCAN_MM


def _m_to_mm(metros: float) -> float:
    """Metros de scan_path → milímetros para mostrar."""
    return metros * _SCAN_MM


def _mm_to_spin(metros: float) -> int:
    """Metros de un YAML guardado → milímetros del spin box."""
    return int(round(metros * _SCAN_MM))


def _finite(valor: float) -> bool:
    """True si el valor es un número finito (distancias pueden ser inf)."""
    return valor == valor and valor not in (float("inf"), float("-inf"))


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
# Tolerancia (rad) para considerar un waypoint "completado" en el feedback del
# goal de trayectoria. Es SOLO visual (la GUI no decide el movimiento): cuando
# feedback.desired coincide con las q de un waypoint se muestra "✓ name completado".
_WAYPOINT_FEEDBACK_TOL = 0.02
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
        self._wp_pending: list[tuple[str, list[float]]] | None = None  # feedback por puntos
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

        # ── TRES columnas principales (apaisado) ──────────────────────
        # El barrido de escaneo tiene su propia columna completa: la tabla
        # P1/P2/P3 (9 columnas) necesita ancho real, y encajarla entre los
        # puntos de inspección y los puntos del usuario la dejaba ilegible.
        cols = QHBoxLayout()
        cols.setSpacing(12)
        root.addLayout(cols, 1)

        # Columna 1: visión (cámara + captura de la posición actual).
        left = QVBoxLayout()
        left.setSpacing(10)
        cols.addLayout(left, 4)

        # Columna 2: operación del robot (control, puntos, manual, traj.).
        right = QVBoxLayout()
        right.setSpacing(10)
        cols.addLayout(right, 5)

        # Columna 3: barrido de escaneo (configuración + tabla derivada).
        scan_col = QVBoxLayout()
        scan_col.setSpacing(10)
        cols.addLayout(scan_col, 6)

        # ── Barra de control superior (compacta, altura fija) ─────────
        top_box = _make_box("Control del robot")
        top_box.setFixedHeight(58)
        hb = QHBoxLayout(top_box)
        hb.setContentsMargins(10, 2, 10, 6)
        hb.setSpacing(6)
        self._status_label = QLabel("● CONECTANDO...")
        self._status_label.setStyleSheet(_label_style(_GRIS_MED, _FAMILIA, 13))
        self._status_label.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        self._status_label.setMinimumWidth(315)
        self._status_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        hb.addWidget(self._status_label, 1)
        self.btn_enable = _make_btn(
            "ENABLE", _btn_style(_VERDE, font_size=12, min_h=24, pad="2px 6px")
        )
        self.btn_disable = _make_btn(
            "DISABLE", _btn_style(_GRIS_MED, font_size=12, min_h=24, pad="2px 6px")
        )
        self.btn_enable.setFixedSize(68, 30)
        self.btn_disable.setFixedSize(74, 30)
        hb.addWidget(self.btn_enable)
        hb.addWidget(self.btn_disable)
        self.btn_home = _make_btn(
            "HOME", _btn_style(_AZUL, font_size=12, min_h=24, pad="2px 6px")
        )
        self.btn_home.setFixedSize(56, 30)
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
        manual_layout = self._frame_panel.layout()
        summary_layout = manual_layout.itemAt(manual_layout.count() - 1).layout()
        summary_layout.setSizeConstraint(QLayout.SetFixedSize)
        for valor in self._frame_panel._summary_labels.values():
            valor.setStyleSheet(
                f"color: {_GRIS_CLR}; font-family: {_FAM_MONO};"
                f" font-size: 12px; background-color: {_NEGRO};"
                f" border: 1px solid {_GRIS_MED}; border-radius: 3px;"
                f" padding: 1px 3px; min-width: 40px; max-width: 40px;"
            )
            valor.setFixedSize(48, 24)
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
        self.btn_p1 = _make_btn(
            "P1", _btn_style(_GRIS_MED, font_size=12, min_w=46, min_h=24, pad="3px 8px")
        )
        self.btn_p2 = _make_btn(
            "P2", _btn_style(_GRIS_MED, font_size=12, min_w=46, min_h=24, pad="3px 8px")
        )
        self.btn_p3 = _make_btn(
            "P3", _btn_style(_GRIS_MED, font_size=12, min_w=46, min_h=24, pad="3px 8px")
        )
        for button in (self.btn_p1, self.btn_p2, self.btn_p3):
            button.setFixedSize(58, 30)
        hb_puntos.addStretch()
        hb_puntos.addWidget(self.btn_p1)
        hb_puntos.addWidget(self.btn_p2)
        hb_puntos.addWidget(self.btn_p3)
        hb_puntos.addStretch()
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
        self._points_list.setMinimumHeight(90)
        v_user.addWidget(self._points_list, 1)
        up_btns = QHBoxLayout()
        up_btns.setContentsMargins(0, 0, 0, 0)
        up_btns.setSpacing(4)
        self.btn_new_point = _make_btn(
            "NUEVO PUNTO", _btn_style(_AZUL, font_size=11, min_h=28, pad="4px 8px")
        )
        self.btn_edit_point = _make_btn(
            "EDITAR", _btn_style(_GRIS_MED, font_size=11, min_h=28, pad="4px 8px")
        )
        self.btn_delete_point = _make_btn(
            "ELIMINAR", _btn_style(_ROJO, font_size=11, min_h=28, pad="4px 8px")
        )
        self.btn_move_point = _make_btn(
            "MOVER A PUNTO", _btn_style(_VERDE, font_size=11, min_h=28, pad="4px 8px")
        )
        self.btn_add_to_traj = _make_btn(
            "AÑADIR A TRAYECTORIA",
            _btn_style(_AZUL, font_size=11, min_h=28, pad="4px 8px"),
        )
        for button in (
            self.btn_new_point,
            self.btn_edit_point,
            self.btn_delete_point,
            self.btn_move_point,
            self.btn_add_to_traj,
        ):
            button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            up_btns.addWidget(button, 1)
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
        self._trajectory_list.setMinimumHeight(90)
        v_traj.addWidget(self._trajectory_list, 1)
        tuv_btns = QHBoxLayout()
        tuv_btns.setContentsMargins(0, 0, 0, 0)
        tuv_btns.setSpacing(4)
        self.btn_traj_up = _make_btn(
            "↑", _btn_style(_GRIS_MED, font_size=12, min_h=28, min_w=38, pad="2px 8px")
        )
        self.btn_traj_down = _make_btn(
            "↓", _btn_style(_GRIS_MED, font_size=12, min_h=28, min_w=38, pad="2px 8px")
        )
        self.btn_traj_remove = _make_btn(
            "QUITAR", _btn_style(_ROJO, font_size=11, min_h=28, pad="4px 8px")
        )
        self.btn_traj_clear = _make_btn(
            "LIMPIAR", _btn_style(_GRIS_MED, font_size=11, min_h=28, pad="4px 8px")
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
        jnt_box.setMinimumWidth(300)
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

        # ── BARRIDO DE ESCANEO (columna 3, completa) ─────────────────
        # Los controles solo editan la FUENTE DE VERDAD (cara, distancia,
        # longitud, dirección). P1/P2, su orientación, la IK y los límites los
        # calcula scan_path.py al pulsar REGENERAR.
        self._scan_panel = self._build_scan_panel()
        scan_col.addWidget(self._scan_panel, 1)

        # ── Mensajes / log del sistema (pie de la columna del barrido) ─
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

        # Mensajes: sección final de la columna del barrido (crece con la ventana).
        scan_col.addWidget(log_box, 1)

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

        # ── Barrido: carga el YAML y reconstruye los puntos derivados ──
        self._load_scan_paths()

        # ── Dimensionado: tres columnas apaisadas ──────────────────────
        # El mínimo lo fija la suma de las anchuras mínimas de las tres
        # columnas: si no, el barrido se aplasta y la tabla deja de caber
        # sin desplazamiento horizontal. La ventana sigue redimensionable.
        mh = self.minimumSizeHint()
        min_w = max(1440, mh.width())
        min_h = max(820, mh.height())
        self.setMinimumSize(min_w, min_h)
        self.resize(max(min_w, 1680), max(min_h, 980))

    # ══ PANEL DE BARRIDO ═══════════════════════════════════════════════
    # Toda la geometría, orientación, IK y validación viven en scan_path.py.
    # Aquí solo se editan parámetros de la fuente de verdad, se llama a
    # resolve_scan() y se pinta lo que devuelve. No se duplica ningún cálculo.

    def _build_scan_panel(self) -> QGroupBox:
        """Panel de configuración del barrido + tabla de puntos derivados."""
        box = _make_box("Barrido de escaneo (geométrico, sin movimiento)")
        v = QVBoxLayout(box)
        v.setSpacing(6)

        # ── Rejilla de controles ─────────────────────────────────────
        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(5)
        grid.setColumnStretch(1, 1)

        def fila(row: int, texto: str, widget: QWidget) -> QLabel:
            lbl = QLabel(texto)
            lbl.setStyleSheet(_label_style(_BLANCO_HUESO, _FAM_MONO, 12))
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            grid.addWidget(lbl, row, 0)
            grid.addWidget(widget, row, 1)
            return lbl

        # Cara: las seis del cubo. El orden lo fija scan_path.FACE_NORMALS.
        self._scan_face_combo = QComboBox()
        self._scan_face_combo.addItems(list(FACE_NORMALS.keys()))
        self._scan_face_combo.setStyleSheet(self._scan_input_style())
        fila(0, "Cara:", self._scan_face_combo)

        # Distancias y longitudes en MILÍMETROS (scan_path trabaja en metros).
        # Arranca en la distancia óptima de trabajo (325 mm); la línea verde
        # de la vista de cámara usa este mismo valor como referencia.
        self._scan_distance_spin = self._scan_mm_spin(
            _SCAN_DIST_DEFAULT_MM, _SCAN_DIST_MIN_MM, _SCAN_DIST_MAX_MM,
            _SCAN_DIST_STEP_MM,
            "mm de distancia de la cámara a la MESA (plano de apoyo), no a "
            "la cara superior de la pieza",
        )
        fila(1, "Scan distance:", self._scan_distance_spin)

        self._scan_length_spin = self._scan_mm_spin(
            80, _SCAN_LEN_MIN_MM, _SCAN_LEN_MAX_MM, _SCAN_LEN_STEP_MM,
            "Longitud NOMINAL de la franja a inspeccionar (informativa). El "
            "recorrido real P1->P2 NO depende de este valor: lo fijan los "
            "bordes REALES de la pieza más los márgenes de inicio y fin",
        )
        fila(2, "Longitud barrido:", self._scan_length_spin)

        self._scan_margin_start_spin = self._scan_mm_spin(
            _SCAN_MARGIN_START_DEFAULT_MM,
            _SCAN_MARGIN_MIN_MM, _SCAN_MARGIN_MAX_MM, _SCAN_MARGIN_STEP_MM,
            "mm que se ANTICIPA el inicio del barrido: P1 se retranquea hacia "
            "FUERA, antes del borde de entrada de la pieza (lado de arranque, "
            "según el sentido configurado)",
        )
        fila(3, "Margen inicio:", self._scan_margin_start_spin)

        self._scan_margin_end_spin = self._scan_mm_spin(
            _SCAN_MARGIN_END_DEFAULT_MM,
            _SCAN_MARGIN_MIN_MM, _SCAN_MARGIN_MAX_MM, _SCAN_MARGIN_STEP_MM,
            "mm que se APLAZA el final del barrido: P2 se avanza hacia "
            "FUERA, después del borde opuesto de la pieza (lado de llegada, "
            "según el sentido configurado)",
        )
        fila(4, "Margen final:", self._scan_margin_end_spin)

        # Nombre del barrido (identificador en el YAML, no un punto).
        self._scan_name_edit = QLineEdit("superior_h")
        self._scan_name_edit.setStyleSheet(self._scan_input_style())
        self._scan_name_edit.setPlaceholderText("nombre del barrido")
        fila(5, "Nombre:", self._scan_name_edit)

        v.addLayout(grid)

        # ── Dirección del barrido (editable, con botón de selección) ──
        dir_row = QHBoxLayout()
        dir_row.setSpacing(4)
        dir_lbl = QLabel("Dirección:")
        dir_lbl.setStyleSheet(_label_style(_BLANCO_HUESO, _FAM_MONO, 12))
        dir_row.addWidget(dir_lbl)
        self._scan_axis_combo = QComboBox()
        for etiqueta, eje in _SCAN_DEFAULT_AXIS.items():
            self._scan_axis_combo.addItem(
                f"[{eje[0]:g}, {eje[1]:g}, {eje[2]:g}]", eje
            )
        self._scan_axis_combo.setStyleSheet(self._scan_input_style())
        self._scan_axis_combo.setToolTip(
            "Eje de la línea de barrido en base_link. Solo direcciones "
            "tangentes a la cara; la geometría lo proyecta sobre el plano."
        )
        dir_row.addWidget(self._scan_axis_combo, 1)
        self._scan_axis_default_btn = _make_btn(
            "DEFECTO",
            _btn_style(_GRIS_MED, font_size=10, min_h=24, pad="2px 8px"),
        )
        self._scan_axis_default_btn.setToolTip(
            "Restablece la dirección por defecto de la cara seleccionada."
        )
        dir_row.addWidget(self._scan_axis_default_btn)
        v.addLayout(dir_row)

        # ── Botones de acción ────────────────────────────────────────
        btn_row = QHBoxLayout()
        btn_row.setSpacing(4)
        self.btn_scan_regenerate = _make_btn(
            "REGENERAR",
            _btn_style(_AZUL, font_size=12, min_h=30, pad="4px 12px"),
        )
        self.btn_scan_regenerate.setToolTip(
            "Recalcula P1/P2, orientaciones, IK y límites con la "
            "configuración actual."
        )
        self.btn_scan_save = _make_btn(
            "GUARDAR",
            _btn_style(_VERDE, font_size=12, min_h=30, pad="4px 12px"),
        )
        self.btn_scan_save.setToolTip(
            "Guarda la fuente de verdad en scan_paths.yaml (los puntos y la "
            "validación NO se guardan: se regeneran)."
        )
        self.btn_scan_reload = _make_btn(
            "RECARGAR",
            _btn_style(_GRIS_MED, font_size=12, min_h=30, pad="4px 12px"),
        )
        for btn in (self.btn_scan_regenerate, self.btn_scan_save, self.btn_scan_reload):
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            btn_row.addWidget(btn, 1)
        v.addLayout(btn_row)

        # ── EJECUTAR BARRIDO (fila propia: es la acción de riesgo) ───
        # El barrido es solo P1 -> P2: la trayectoria termina en P2, sin
        # segmento de retirada posterior.
        self.btn_scan_execute = _make_btn(
            "EJECUTAR BARRIDO",
            _btn_style(_ROJO_CLARO, font_size=13, min_h=34, pad="6px 12px"),
        )
        self.btn_scan_execute.setToolTip(
            "Ejecuta el barrido P1 -> P2 con las q ya resueltas por "
            "scan_path, y termina en P2. Sin densificar y sin velocidades: "
            "el JTC interpola linealmente en espacio articular. Solo se "
            "habilita con el robot ya en P1 (dentro del umbral)."
        )
        self.btn_scan_execute.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.btn_scan_execute.setEnabled(False)
        v.addWidget(self.btn_scan_execute)

        # ── IR A P1: APROXIMACIÓN, goal independiente del barrido ──────
        # El barrido empieza en P1(t=0): si el robot está en otro sitio, el
        # primer comando se recorta y el goal aborta. Este botón lleva el
        # robot a P1 por separado y no forma parte del barrido P1 -> P2.
        self.btn_scan_goto_p1 = _make_btn(
            "IR A P1",
            _btn_style(_AZUL, font_size=13, min_h=34, pad="6px 12px"),
        )
        self.btn_scan_goto_p1.setToolTip(
            "Aproximación independiente: lleva el robot desde su posición "
            "actual hasta P1 con un goal propio (tiempo según la distancia "
            "articular). No forma parte del barrido; solo habilita "
            "EJECUTAR BARRIDO."
        )
        self.btn_scan_goto_p1.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.btn_scan_goto_p1.setEnabled(False)
        v.addWidget(self.btn_scan_goto_p1)

        # ── Posición real respecto a P1 (por qué está bloqueado) ───────
        self._scan_goto_status = QLabel("Sin calcular: pulsa REGENERAR")
        self._scan_goto_status.setAlignment(Qt.AlignCenter)
        self._scan_goto_status.setWordWrap(True)
        self._scan_goto_status.setStyleSheet(
            _label_style(
                _AMARILLO, _FAMILIA, 12,
                extra=f"background-color: {_NEGRO}; border: 1px solid {_AMARILLO};"
                      " border-radius: 4px; padding: 4px 8px;",
            )
        )
        v.addWidget(self._scan_goto_status)

        # ── Veredicto global (verde/rojo muy visible) ────────────────
        self._scan_verdict = QLabel("Sin calcular: pulsa REGENERAR")
        self._scan_verdict.setAlignment(Qt.AlignCenter)
        self._scan_verdict.setWordWrap(True)
        self._scan_verdict.setStyleSheet(
            _label_style(
                _AMARILLO, _FAMILIA, 13,
                extra=f"background-color: {_NEGRO}; border: 2px solid {_AMARILLO};"
                      " border-radius: 4px; padding: 5px 8px;",
            )
        )
        v.addWidget(self._scan_verdict)

        # ── Tabla P1/P2 ──────────────────────────────────────────────
        self._scan_table = QTableWidget(len(ROLES), len(_SCAN_TABLE_HEADERS))
        self._scan_table.setHorizontalHeaderLabels(list(_SCAN_TABLE_HEADERS))
        self._scan_table.verticalHeader().setVisible(False)
        self._scan_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._scan_table.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self._scan_table.setFocusPolicy(Qt.NoFocus)
        self._scan_table.setStyleSheet(
            "QTableWidget {"
            f" background-color: {_NEGRO}; color: {_BLANCO_HUESO};"
            f" gridline-color: {_GRIS_MED};"
            " border: 1px solid #c0392b; border-radius: 4px;"
            f" font-family: {_FAM_MONO}; font-size: 11px; }}"
            "QTableWidget::item { padding: 3px 4px; }"
            f"QHeaderView::section {{ background-color: {_GRIS_OSC};"
            f" color: {_GRIS_CLR}; border: none; padding: 4px 3px;"
            " font-weight: bold; font-size: 11px; }"
        )
        header = self._scan_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        header.setMinimumSectionSize(38)
        # La tabla crece con la columna y ya no tiene techo: en su propia
        # columna hay sitio de sobra para las 3 filas sin apretar.
        self._scan_table.setMinimumHeight(150)
        self._scan_table.verticalHeader().setDefaultSectionSize(30)
        v.addWidget(self._scan_table, 1)

        # Celdas vacías desde el principio: la tabla siempre es inspeccionable,
        # incluso antes del primer REGENERAR.
        for row in range(len(ROLES)):
            for col in range(len(_SCAN_TABLE_HEADERS)):
                self._scan_table.setItem(row, col, QTableWidgetItem("—"))

        # Ancho mínimo del panel = lo que la tabla necesita para que sus 9
        # columnas quepan enteras. Así no aparece desplazamiento horizontal y
        # el mínimo de la ventana sube solo si hiciera falta.
        tabla_px = self._scan_table.sizeHint().width() + 34
        box.setMinimumWidth(max(430, tabla_px))

        self._scan_rows = {}  # role -> fila de la tabla
        self._scan_solution = None  # última ScanSolution derivada
        self._scan_store = None  # ScanPathStore

        # Conexiones del panel.
        self._scan_face_combo.currentTextChanged.connect(self._on_scan_face_changed)
        self._scan_axis_default_btn.clicked.connect(self._on_scan_axis_default)
        self.btn_scan_regenerate.clicked.connect(self._on_scan_regenerate)
        self.btn_scan_save.clicked.connect(self._on_scan_save)
        self.btn_scan_reload.clicked.connect(self._load_scan_paths)
        self.btn_scan_execute.clicked.connect(self._on_scan_execute)
        self.btn_scan_goto_p1.clicked.connect(self._on_scan_goto_p1)

        return box

    def _scan_input_style(self) -> str:
        """Estilo compartido por los controles de entrada del panel."""
        return (
            f"QComboBox, QSpinBox, QLineEdit {{"
            f" background-color: {_NEGRO}; color: {_AMARILLO_BRILLANTE};"
            f" border: 2px solid {_AMARILLO}; border-radius: 4px;"
            f" padding: 3px 6px; font-family: {_FAM_MONO}; font-size: 13px;"
            " font-weight: bold; }"
            f"QComboBox:hover, QSpinBox:hover, QLineEdit:hover"
            f" {{ border-color: {_AMARILLO_BRILLANTE}; }}"
        )

    def _scan_mm_spin(
        self, value: int, minimum: int, maximum: int, step: int, tip: str
    ) -> QSpinBox:
        """Spin box en milímetros; scan_path sigue trabajando en metros."""
        spin = QSpinBox()
        spin.setRange(minimum, maximum)
        spin.setSingleStep(step)
        spin.setValue(value)
        spin.setSuffix(" mm")
        spin.setToolTip(tip)
        spin.setStyleSheet(self._scan_input_style())
        return spin

    # ── Barrido: ciclo de vida (carga / regenerate / guardar) ──────────

    def _load_scan_paths(self) -> None:
        """Carga scan_paths.yaml y reconstruye P1/P2 a partir de él.

        Si el YAML contiene una configuración no alcanzable, se muestra como
        inválida: nunca se descarta en silencio.
        """
        self._scan_store = ScanPathStore()
        for err in self._scan_store.errors:
            self._log("warning", f"[BARRIDO] {err}")

        profile = self._scan_store.profile
        # La línea verde de la cámara usa la MISMA distancia de trabajo que el
        # perfil cargado (aunque no haya barridos guardados), para que nunca
        # diverjan el valor mostrado y la referencia proyectada.
        self._scan_distance_spin.setValue(_mm_to_spin(profile.scan_distance))

        specs = self._scan_store.all()
        if not specs:
            self._scan_solution = None
            self._clear_scan_table()
            self._set_scan_verdict(
                "Sin barrido guardado: ajusta la configuración y pulsa REGENERAR",
                _AMARILLO,
            )
            return

        spec = specs[0]
        # Rellena los controles con la fuente de verdad guardada.
        self._scan_face_combo.setCurrentText(spec.face)
        self._scan_distance_spin.setValue(_mm_to_spin(profile.scan_distance))
        self._scan_length_spin.setValue(_mm_to_spin(spec.line_length))
        self._scan_margin_start_spin.setValue(_mm_to_spin(spec.scan_margin_start))
        self._scan_margin_end_spin.setValue(_mm_to_spin(spec.scan_margin_end))
        self._scan_name_edit.setText(spec.name)
        self._select_scan_axis(spec.line_axis)

        self._log(
            "info",
            f"[BARRIDO] '{spec.name}' ({spec.face}) cargado desde "
            f"{self._scan_store.path.name}",
        )
        self._on_scan_regenerate()

    def _on_scan_face_changed(self, face: str) -> None:
        """Propone la dirección por defecto de la cara elegida."""
        if face in _SCAN_DEFAULT_AXIS:
            self._select_scan_axis(_SCAN_DEFAULT_AXIS[face])

    def _on_scan_axis_default(self) -> None:
        """Botón DEFECTO: vuelve al eje recomendado para la cara actual."""
        face = self._scan_face_combo.currentText()
        if face in _SCAN_DEFAULT_AXIS:
            self._select_scan_axis(_SCAN_DEFAULT_AXIS[face])

    def _select_scan_axis(self, axis) -> None:
        """Elige en el combo el eje equivalente a `axis` (por coincidencia)."""
        objetivo = tuple(float(v) for v in axis)
        for idx in range(self._scan_axis_combo.count()):
            if tuple(self._scan_axis_combo.itemData(idx)) == objetivo:
                self._scan_axis_combo.setCurrentIndex(idx)
                return
        # Eje no catalogado (p. ej. editado a mano en el YAML): se añade.
        self._scan_axis_combo.addItem(
            f"[{objetivo[0]:g}, {objetivo[1]:g}, {objetivo[2]:g}]", objetivo
        )
        self._scan_axis_combo.setCurrentIndex(self._scan_axis_combo.count() - 1)

    def _scan_current_spec(self) -> ScanPathSpec:
        """Construye el ScanPathSpec desde los controles de la GUI (metros)."""
        nombre = self._scan_name_edit.text().strip() or "barrido"
        axis = self._scan_axis_combo.currentData() or (1.0, 0.0, 0.0)
        return ScanPathSpec(
            name=nombre,
            face=self._scan_face_combo.currentText(),
            line_length=_spin_to_m(self._scan_length_spin.value()),
            line_axis=[float(v) for v in axis],
            scan_margin_start=_spin_to_m(self._scan_margin_start_spin.value()),
            scan_margin_end=_spin_to_m(self._scan_margin_end_spin.value()),
        )

    def _scan_current_profile(self) -> ScanProfile:
        """Construye el ScanProfile desde los controles de la GUI (metros)."""
        # La pieza se conserva tal cual esté en el almacén (o el defecto).
        part = self._scan_store.profile.part if self._scan_store else PartBox()
        return ScanProfile(
            part=PartBox(center=list(part.center), size=list(part.size)),
            scan_distance=_spin_to_m(self._scan_distance_spin.value()),
        )

    def _on_scan_regenerate(self) -> None:
        """REGENERAR: geometría + orientación + IK + límites vía scan_path."""
        try:
            spec = self._scan_current_spec()
            profile = self._scan_current_profile()
        except ValueError as exc:
            self._log("error", f"[BARRIDO] configuración inválida ({exc})")
            self._set_scan_verdict(f"✗ CONFIGURACIÓN INVÁLIDA — {exc}", _ROJO)
            return

        try:
            # Cinemática nueva por resolución: el solver multi-start sortea
            # semillas y reutilizar la instancia haría el resultado variable.
            solution = resolve_scan(profile, spec, load_kinematics())
        except Exception as exc:  # noqa: BLE001 - la GUI no debe romperse
            self._log("error", f"[BARRIDO] no se pudo calcular ({exc})")
            self._set_scan_verdict(f"✗ ERROR AL CALCULAR — {exc}", _ROJO)
            return

        self._scan_solution = solution
        self._render_scan_table(solution)
        self._verdict_for(solution)

        if solution.valid:
            self._log(
                "ok",
                f"[BARRIDO] {spec.face} válido: línea "
                f"{_m_to_mm(solution.scan_length):.0f} mm a "
                f"{_m_to_mm(profile.scan_distance):.0f} mm",
            )
        else:
            self._log(
                "warning",
                f"[BARRIDO] {spec.face} NO válido "
                f"(articulaciones: {solution.offending_joints or '—'})",
            )

    def _on_scan_save(self) -> None:
        """GUARDAR: persiste solo la fuente de verdad (nunca P1/P2)."""
        if self._scan_store is None:
            self._scan_store = ScanPathStore()
        try:
            spec = self._scan_current_spec()
            profile = self._scan_current_profile()
        except ValueError as exc:
            self._log("error", f"[BARRIDO] no se puede guardar ({exc})")
            return

        try:
            self._scan_store.set_part(profile.part)
            self._scan_store.set_scan_distance(profile.scan_distance)
            self._scan_store.update(spec)
            self._scan_store.save()
        except (OSError, ValueError) as exc:
            self._log("error", f"[BARRIDO] no se pudo guardar ({exc})")
            return
        self._log("ok", f"[BARRIDO] guardado '{spec.name}' en {self._scan_store.path}")

    # ── Barrido: ejecución ─────────────────────────────────────────────

    def _scan_p1_q(self) -> list[float] | None:
        """q de P1 tal como la resolvió scan_path, o None si no hay barrido."""
        solution = getattr(self, "_scan_solution", None)
        if solution is None or not solution.valid:
            return None
        point = solution.point("P1")
        if point is None:
            return None
        return [float(v) for v in point.q]

    def _scan_distance_to_p1(self) -> float | None:
        """Separación articular MÁXIMA (rad) entre el robot medido y P1.

        Es la misma métrica por la que el joint_trajectory_controller valida su
        `goal_tolerance` (por articulación), así que "dentro del umbral" quiere
        decir exactamente "el goal del barrido arrancará dentro de tolerancia".

        Devuelve None si no hay solución válida o si aún no han llegado las 6
        articulaciones: entonces no se puede afirmar nada, y el barrido queda
        bloqueado por prudencia en lugar de arriesgarse.
        """
        q_now = self._current_joint_q()
        q_p1 = self._scan_p1_q()
        if q_now is None or q_p1 is None:
            return None
        return max(abs(a - b) for a, b in zip(q_now, q_p1))

    def _set_scan_goto_status(self, text: str, color: str) -> None:
        """Pinta el estado de la aproximación (idempotente: llega a ~30 Hz)."""
        label = getattr(self, "_scan_goto_status", None)
        if label is None:
            return
        mismo_color = getattr(self, "_scan_goto_status_color", None) == color
        if label.text() == text and mismo_color:
            return
        self._scan_goto_status_color = color
        label.setText(text)
        label.setStyleSheet(
            _label_style(
                color, _FAMILIA, 12,
                extra=f"background-color: {_NEGRO}; border: 1px solid {color};"
                      " border-radius: 4px; padding: 4px 8px;",
            )
        )

    def _update_scan_buttons(self, *, en_ejecucion: bool = False) -> None:
        """ÚNICA fuente de verdad de los botones IR A P1 / EJECUTAR BARRIDO.

        La regla es una sola y explícita:

            P1 es válido y el robot está a <= _SCAN_AT_P1_TOL_RAD de P1
                -> EJECUTAR BARRIDO habilitado (IR A P1 no hace falta)
            P1 es válido y el robot está más lejos
                -> IR A P1 habilitado, EJECUTAR BARRIDO bloqueado
            sin solución válida o sin /joint_states completo
                -> ambos bloqueados

        Se invoca desde todos los sitios que pueden cambiar el resultado:
        cada `/joint_states`, REGENERAR, cualquier veredicto y al terminar
        cualquier movimiento (incluido el propio IR A P1).
        """
        if not (
            hasattr(self, "btn_scan_execute") and hasattr(self, "btn_scan_goto_p1")
        ):
            return
        ejecutando = en_ejecucion or bool(getattr(self, "_motion_active", False))
        distancia = self._scan_distance_to_p1()

        if distancia is None:
            self.btn_scan_execute.setEnabled(False)
            self.btn_scan_goto_p1.setEnabled(False)
            if self._scan_p1_q() is None:
                self._set_scan_goto_status(
                    "— Sin barrido válido: pulsa REGENERAR", _AMARILLO
                )
            else:
                self._set_scan_goto_status(
                    "— Esperando /joint_states (no se puede comprobar P1)", _AMARILLO
                )
            return

        if ejecutando:
            self.btn_scan_execute.setEnabled(False)
            self.btn_scan_goto_p1.setEnabled(False)
            self._set_scan_goto_status("● Movimiento en curso…", _AMARILLO)
            return

        en_p1 = distancia <= _SCAN_AT_P1_TOL_RAD
        self.btn_scan_execute.setEnabled(en_p1)
        self.btn_scan_goto_p1.setEnabled(not en_p1)
        if en_p1:
            self._set_scan_goto_status(
                f"● EN P1 (Δ {distancia:.4f} rad ≤ {_SCAN_AT_P1_TOL_RAD:g}) — "
                "listo para barrer",
                _VERDE,
            )
        else:
            self._set_scan_goto_status(
                f"● FUERA de P1 (Δ {distancia:.4f} rad > {_SCAN_AT_P1_TOL_RAD:g}) — "
                "pulsa IR A P1 primero",
                _AMARILLO,
            )

    def _scan_goto_p1_duration(self, q_now, q_dest) -> float:
        """`time_from_start` (s) para el waypoint único de IR A P1.

        Es el tiempo de un movimiento trapezoidal por ARTICULACIÓN, tomando el
        máximo de las seis: la goal la tiene que cumplir la más lenta, no la
        media ni la primera.

            v = fracción · max_velocity(j)
            a = fracción · max_acceleration(j)
            t_acc = v / a
            t_cru = max(0, |Δq_j| − v²/a) / v
            T_j    = 2·t_acc + t_cru
            T      = seguridad · max_j(T_j)

        Por cada articulación se comparan las dos cotas posibles: la de
        velocidad pura (alcanzar el tope en el menor tiempo) y la de aceleración
        (subir y frenar dentro de la ventana). La fórmula trapezoidal ya las
        cubre: si |Δq| es pequeño frente a v²/a degenera en triangular, y
        si es grande añade el tramo de velocidad constante.

        `T` es solo la DIMENSIÓN del reloj que se le pasa al JTC. No se manda
        ningún perfil: el controlador traza su propio spline desde el estado
        medido hasta P1 dentro de ese tiempo.

        Añade un corrector suave basado en la holgura al soft limit: cuando
        alguna articulación está muy cerca de su soft limit en el objetivo,
        se aumenta la duración para dar tiempo a que el JointSoftLimiter complete
        el movimiento (sin modificar tolerancias ni soft limits).
        """
        Delta = [abs(float(dest) - float(now)) for now, dest in zip(q_now, q_dest)]

        peor_t = 0.0
        for j, dq in enumerate(Delta):
            v = _SCAN_GOTO_P1_VEL_FRACTION * _SCAN_GOTO_P1_MAX_VEL_RAD_S[j]
            a = _SCAN_GOTO_P1_ACC_FRACTION * _SCAN_GOTO_P1_MAX_ACC_RAD_S2[j]
            if v <= 0.0 or a <= 0.0:
                continue
            # Distancia cubierta mientras acelera: v²/a. Si |Δq| es menor, el
            # perfil es triangular (no llega a la velocidad de crucero).
            t_acc = v / a
            t_cru = max(0.0, dq - v * t_acc) / v
            t_j = 2.0 * t_acc + t_cru
            peor_t = max(peor_t, t_j)

        # Corrector por holgura al soft limit (especialmente J5)
        peor_f = 1.0
        alpha = 0.01  # factor suave
        eps_h = 1e-4
        f_max = 3.0
        qd = [float(x) for x in q_dest]
        for j, qj in enumerate(qd):
            slo, shi = _SCAN_GOTO_P1_SOFT_LOWER[j], _SCAN_GOTO_P1_SOFT_UPPER[j]
            holgura = min(qj - slo, shi - qj)
            if holgura < 0.0:
                holgura = 0.0  # objetivo fuera de soft; ya filtrado por validación
            f_j = 1.0 + alpha / max(holgura, eps_h)
            if f_j > f_max:
                f_j = f_max
            if f_j > peor_f:
                peor_f = f_j

        duracion = peor_t * peor_f * _SCAN_GOTO_P1_TIME_SAFETY
        return max(
            _SCAN_GOTO_P1_MIN_S, min(_SCAN_GOTO_P1_MAX_S, duracion)
        )
    def _on_scan_goto_p1(self) -> None:
        """IR A P1: goal de APROXIMACIÓN, independiente del barrido.

        Envía únicamente q_medida -> P1, como una trayectoria trapezoidal
        multi-waypoint con su propio goal. No forma parte del barrido P1 -> P2:
        no se registra como escaneo y no toca los tiempos del barrido. Al
        terminar, `/joint_states` confirma la llegada y EJECUTAR BARRIDO queda
        habilitado.
        """
        if self._motion_active:
            return
        if self._scan_solution is None:
            self._log("error", "[P1] no hay solución: pulsa REGENERAR primero")
            return
        if not self._scan_solution.valid:
            self._log(
                "error",
                f"[P1] solución NO válida (articulaciones: "
                f"{self._scan_solution.offending_joints or '—'}): no se va a P1",
            )
            self._set_scan_verdict("✗ BARRIDO NO VÁLIDO — no se va a P1", _ROJO)
            return

        q_now = self._current_joint_q()
        if q_now is None:
            self._log(
                "error",
                "[P1] sin /joint_states completo: no se puede calcular la aproximación",
            )
            return

        distancia = self._scan_distance_to_p1()
        q_p1 = self._scan_p1_q()
        duracion = self._scan_goto_p1_duration(q_now, q_p1)
        self._log(
            "info",
            f"[P1] APROXIMACIÓN (no forma parte del barrido): Δ {distancia:.4f} rad "
            f"→ UN waypoint en {duracion:.2f} s "
            f"({_SCAN_GOTO_P1_VEL_FRACTION:.0%} del límite articular, "
            f"x{_SCAN_GOTO_P1_TIME_SAFETY:g} de seguridad)",
        )
        self._log(
            "info",
            f"[P1]   desde [{', '.join(f'{v:+.4f}' for v in q_now)}]",
        )
        self._log(
            "info",
            f"[P1]   hasta  [{', '.join(f'{v:+.4f}' for v in q_p1)}]",
        )

        goal = self._build_single_goal(
            q_p1, duracion, time_tolerance_s=_SCAN_GOTO_P1_TIME_TOL_S
        )
        self._set_scan_verdict("▶ IR A P1: APROXIMANDO...", _AMARILLO)
        self._run_goal(
            goal=goal,
            button=self.btn_scan_goto_p1,
            moving_text="● IR A P1: APROXIMANDO...",
            ok_text="● EN P1 (aproximación completada)",
            tag="IR A P1",
            fail_text="● IR A P1: APROXIMACIÓN FALLIDA",
        )

    def _on_scan_execute(self) -> None:
        """EJECUTAR BARRIDO: P1 -> P2, un único segmento de escaneo.

        No recalcula la geometría ni la IK: usa el `q` que `scan_path` ya
        resolvió con el eje óptico real de la cámara, lo planifica
        `scan_execution` y lo envía como un único goal. Al no emitir
        velocidades, el joint_trajectory_controller interpola linealmente en
        espacio articular, así que el tramo P1 -> P2 NO es la recta cartesiana
        (se aparta ~1.4 mm): es justo lo que esta v1 quiere observar.
        """
        if self._motion_active:
            return

        solution = self._scan_solution
        if solution is None:
            self._log("error", "[BARRIDO] no hay solución: pulsa REGENERAR primero")
            self._set_scan_verdict("✗ SIN CÁLCULO — pulsa REGENERAR", _ROJO)
            return
        if not solution.valid:
            self._log(
                "error",
                f"[BARRIDO] solución NO válida (articulaciones: "
                f"{solution.offending_joints or '—'}): no se ejecuta",
            )
            self._set_scan_verdict("✗ BARRIDO NO VÁLIDO — no se ejecuta", _ROJO)
            return

        # ── Aproximación previa: el barrido arranca en P1(t=0) ────────
        # El goal empieza en P1, así que exige estar en P1. Ejecutar desde
        # cualquier otra configuración hace que el soft-limiter recorte el
        # primer comando y el goal muera con GOAL_TOLERANCE_VIOLATED: la
        # separación es un movimiento propio (IR A P1), nunca parte del barrido.
        distancia_p1 = self._scan_distance_to_p1()
        if distancia_p1 is None:
            self._log(
                "error",
                "[BARRIDO] sin /joint_states: no se puede confirmar que el robot "
                "esté en P1",
            )
            self._set_scan_verdict(
                "✗ SIN /joint_states — no se puede verificar la posición en P1", _ROJO
            )
            return
        if distancia_p1 > _SCAN_AT_P1_TOL_RAD:
            self._log(
                "error",
                f"[BARRIDO] el robot está a {distancia_p1:.4f} rad de P1 "
                f"(umbral {_SCAN_AT_P1_TOL_RAD:g}): pulsa IR A P1 primero",
            )
            self._set_scan_verdict(
                f"✗ ROBOT FUERA DE P1 (Δ {distancia_p1:.4f} rad) — pulsa IR A P1",
                _ROJO,
            )
            return

        # ── Plan: ESCANEO P1 -> P2, con q precomputed ──────────────────
        try:
            plan = build_scan_execution(solution)
        except ValueError as exc:
            self._log("error", f"[BARRIDO] no se pudo planificar ({exc})")
            self._set_scan_verdict(f"✗ PLANIFICACIÓN INVÁLIDA — {exc}", _ROJO)
            return

        self._log("info", f"[BARRIDO] {describe_plan(plan)}")
        for linea in plan.summary_lines():
            self._log("info", f"[BARRIDO] {linea}")

        # ── JointTrajectory a partir del q ya resuelto ─────────────────
        try:
            from staubli_trajectory.trajectory_builder import TrajectoryBuilder

            built = TrajectoryBuilder().build_from_joint_waypoints(
                waypoints_to_dicts(plan), self._scan_joint_names()
            )
        except Exception as exc:  # noqa: BLE001 - la GUI no debe romperse
            self._log("error", f"[BARRIDO] error al construir la trayectoria ({exc})")
            self._set_scan_verdict(f"✗ ERROR AL CONSTRUIR — {exc}", _ROJO)
            return

        if built.trajectory is None:
            fuera = [
                r.name for r in built.records if not r.inside_limits
            ]
            self._log(
                "error",
                f"[BARRIDO] waypoints fuera de límites ({', '.join(fuera)}): "
                f"no se envía",
            )
            self._set_scan_verdict("✗ FUERA DE LÍMITES — no se ejecuta", _ROJO)
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = built.trajectory
        self._fill_goal_tolerances(goal)

        self._set_scan_verdict("▶ EJECUTANDO BARRIDO...", _AMARILLO)
        self._run_goal(
            goal=goal,
            button=self.btn_scan_execute,
            moving_text="● BARRIDO: P1 → P2 ESCANEANDO",
            ok_text="● BARRIDO COMPLETADO (escaneo P1 → P2)",
            tag="BARRIDO",
            fail_text="● BARRIDO FALLIDO",
        )

    def _scan_joint_names(self) -> list[str]:
        """Nombres de articulación del barrido: los del JTC, por convención."""
        return [f"joint_{i}" for i in range(1, 7)]

    # ── Barrido: pintado de resultados ─────────────────────────────────

    def _clear_scan_table(self) -> None:
        """Vacia la tabla y devuelve las celdas a un color neutro."""
        normal = QFont()
        for row in range(self._scan_table.rowCount()):
            for col in range(self._scan_table.columnCount()):
                item = self._scan_table.item(row, col)
                if item is None:
                    continue
                item.setText("—")
                item.setBackground(QColor(_NEGRO))
                item.setForeground(QColor(_GRIS_CLR))
                item.setFont(normal)

    def _render_scan_table(self, solution) -> None:
        """Pinta P1/P2 con el sistema verde/rojo ya usado en la GUI."""
        for row, role in enumerate(ROLES):
            point = solution.point(role)
            if point is None:
                for col in range(len(_SCAN_TABLE_HEADERS)):
                    self._scan_table.item(row, col).setText("—")
                continue

            ok = point.solved
            limits = point.inside_limits
            culpable = point.offending_joint_names()
            # Distancia medida por FK al plano de la cara.
            dist_mm = (
                f"{_m_to_mm(point.distance_measured):.1f}"
                if _finite(point.distance_measured)
                else "—"
            )

            valores = (
                point.role,
                f"{point.position_xyz[0]:+.3f}",
                f"{point.position_xyz[1]:+.3f}",
                f"{point.position_xyz[2]:+.3f}",
                ", ".join(f"{v:.2f}" for v in point.rpy_degrees),
                dist_mm,
                "✓" if ok else "✗",
                "✓" if limits else "✗",
                ", ".join(culpable) if culpable else "—",
            )
            verde = point.valid
            fondo = _VERDE if verde else _ROJO
            negrita = QFont()
            negrita.setBold(True)
            normal = QFont()
            for col, texto in enumerate(valores):
                item = self._scan_table.item(row, col)
                item.setText(texto)
                if col >= 6:
                    # Columnas de veredicto (IK / límites / culpable): fondo
                    # verde si el punto es válido, rojo si no. Es el mismo
                    # código de color que ya usa el resto de la GUI.
                    item.setBackground(QColor(fondo))
                    item.setForeground(QColor(_BLANCO))
                    item.setFont(negrita)
                else:
                    item.setBackground(QColor(_NEGRO))
                    item.setForeground(QColor(_BLANCO_HUESO))
                    item.setFont(normal)

    def _verdict_for(self, solution) -> None:
        """Veredicto global: la fila más importante de la interfaz."""
        if solution.valid:
            self._set_scan_verdict(
                "✓ BARRIDO VÁLIDO  (IK y límites correctos en P1 y P2)",
                _VERDE,
            )
            self._set_status("● Barrido: configuración VÁLIDA", _VERDE)
            return

        causas = []
        for point in solution.points:
            if not point.solved:
                causas.append(f"{point.role}: sin solución IK")
            elif not point.inside_limits:
                causas.append(
                    f"{point.role}: límites ({', '.join(point.offending_joint_names())})"
                )
        self._set_scan_verdict(
            f"✗ BARRIDO NO VÁLIDO — {'; '.join(causas) or 'solución inválida'}",
            _ROJO,
        )
        self._set_status("● Barrido: configuración NO VÁLIDA", _ROJO)

    def _set_scan_verdict(self, text: str, color: str) -> None:
        self._scan_verdict.setText(text)
        self._scan_verdict.setStyleSheet(
            _label_style(
                color, _FAMILIA, 13,
                extra=f"background-color: {_NEGRO}; border: 2px solid {color};"
                      " border-radius: 4px; padding: 5px 8px;",
            )
        )
        # El estado de los botones no se decide aquí: lo decide
        # _update_scan_buttons, que es quien conoce la distancia real a P1.
        self._update_scan_buttons(en_ejecucion="EJECUTANDO" in text)

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
        # La distancia a P1 depende del estado medido: al cambiar este, puede
        # cambiar si EJECUTAR BARRIDO / IR A P1 están disponibles.
        self._update_scan_buttons()

    def _current_joint_q(self) -> list[float] | None:
        """q medida ahora en el orden del JTC, o None si aún no se puede saber.

        Única lectura de `self._joints_now` para código de movimiento: devuelve
        None tanto si aún no ha llegado ningún `/joint_states` como si llega
        incompleto (falta alguna de las 6 articulaciones).
        """
        if self._joints_now is None:
            return None
        try:
            return [float(self._joints_now[j]) for j in _HOME_JOINT_NAMES]
        except KeyError:
            return None

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
        self._draw_working_reference(image)
        self._camera_view.set_frame(image)

    def _draw_working_reference(self, image: QImage) -> None:
        """Superpone la línea verde del ancho de FOV a la distancia de trabajo.

        La distancia es la que muestra ahora "Scan distance" (el mismo
        parámetro configurable que la trayectoria y el que carga el perfil
        guardado). La geometría —el segmento del abanico FOV del STL a esa
        distancia, proyectado con la cámara alineada al boresight— la resuelve
        sensor_reference (módulo puro y testeable); aquí sólo se pinta sobre la
        copia de la imagen, sin tocar el mensaje ROS. El segmento puede salirse
        del marco (el abanico es más ancho que el FOV de la cámara): el
        recorte lo hace el propio QPainter.
        """
        spin = getattr(self, "_scan_distance_spin", None)
        if spin is None:
            return
        try:
            distance = _spin_to_m(spin.value())
            p0, p1 = working_reference_pixels(
                distance, image.width(), image.height(), CAMERA_HFOV_RAD
            )
        except Exception:  # noqa: BLE001 - un fallo de dibujo no rompe la GUI
            return
        painter = QPainter(image)
        try:
            pen = QPen(QColor(_VERDE))
            pen.setWidth(2)
            painter.setPen(pen)
            painter.drawLine(QPointF(p0[0], p0[1]), QPointF(p1[0], p1[1]))
        finally:
            painter.end()

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
            ok_text="HOME completado",
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
            ok_text=f"{name} completado",
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

        q_ref = self._current_joint_q()

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
            ok_text=f"{point.name} completado",
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
        waypoints: list[tuple[str, list[float]]] | None = None,
    ) -> None:
        """Envío común HOME/P1-P3/TRAYECTORIA/PUNTO: asegura controlador y sigue.

        Antes de enviar el goal garantiza estar en modo TRAYECTORIA
        (joint_trajectory_controller activo, forward_velocity_controller
        desactivado) cambiando controladores si hiciera falta y, solo cuando
        el cambio se confirma, lanza send_goal_async. 'fail_text', si se da,
        personaliza el mensaje/estado de fallo.

        'waypoints', si se da (puntos de usuario de una trayectoria), activa
        el feedback del goal: cada punto alcanzado se marca como completado
        en el panel de mensajes (solo visual, sin enviar nada al robot).
        """
        if self._motion_active:
            return
        if self._ros_node is None or self._action_client is None or self._ctrl is None:
            print(f"[GUI] {tag} sin ROS (placeholder)")
            return

        self._motion_active = True
        self._wp_pending = list(waypoints) if waypoints else None
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
            feedback_cb = self._on_motion_feedback if self._wp_pending else None
            goal_future = self._action_client.send_goal_async(
                goal, feedback_callback=feedback_cb
            )
            goal_future.add_done_callback(
                lambda fut: self._on_motion_goal_response(
                    fut, button=button, ok_text=ok_text, tag=tag, fail_text=fail_text
                )
            )

        self._switch_to_mode("trayectoria", after=_tras_switch, reason=f"goal {tag}")

    # ── Construcción del goal y seguimiento asíncrono ─────────────────
    def _build_single_goal(
        self,
        positions,
        duration_s: float,
        time_tolerance_s: float = _HOME_GOAL_TIME_TOL_S,
    ) -> FollowJointTrajectory.Goal:
        """Goal de un solo waypoint: trayectoria + tolerancias (como antes).

        Un `JointTrajectoryPoint` con `positions` y `time_from_start`; el
        controlador traza su propio spline desde el estado medido hasta esas
        posiciones dentro de ese tiempo. IR A P1 usa este mismo goal, con su
        propio margen de asentamiento vía `time_tolerance_s`.
        """
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.time_from_start = Duration(seconds=duration_s).to_msg()

        trajectory = JointTrajectory()
        trajectory.joint_names = list(_HOME_JOINT_NAMES)
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        self._fill_goal_tolerances(goal, time_tolerance_s=time_tolerance_s)
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

    def _fill_goal_tolerances(
        self,
        goal: FollowJointTrajectory.Goal,
        time_tolerance_s: float = _HOME_GOAL_TIME_TOL_S,
    ) -> None:
        """Aplica las tolerancias del goal (mismas que staubli_trajectory).

        `time_tolerance_s` permite que IR A P1 use su propio margen de
        asentamiento sin tocar el de HOME ni el del barrido, que se quedan en
        _HOME_GOAL_TIME_TOL_S.
        """
        for name in _HOME_JOINT_NAMES:
            tol = JointTolerance()
            tol.name = name
            tol.position = _HOME_GOAL_TOL_POS
            tol.velocity = _HOME_GOAL_TOL_VEL
            tol.acceleration = _HOME_GOAL_TOL_ACC
            goal.goal_tolerance.append(tol)
        goal.goal_time_tolerance = Duration(
            seconds=time_tolerance_s
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

    def _on_motion_feedback(self, feedback_msg) -> None:
        """Marca cada punto de la trayectoria como completado (solo visual).

        Compara la posición deseada (feedback.desired, el punto exacto que el
        joint_trajectory_controller está siguiendo) con el siguiente waypoint
        pendiente; al coincidir (dentro de _WAYPOINT_FEEDBACK_TOL) registra
        "✓ <name> completado" en el panel. Se usa 'desired' (y no la posición
        medida 'actual') porque la referencia atraviesa exactamente cada
        waypoint del goal, mientras que la posición medida puede no llegar a
        coincidir dentro de la tolerancia. No envía nada al robot: la GUI solo
        muestra el progreso del goal multi-waypoint (que reporta un único
        resultado al final).
        """
        if not self._motion_active or not self._wp_pending:
            return
        feedback = getattr(feedback_msg, "feedback", None)
        if feedback is None:
            return
        desired = list(getattr(feedback.desired, "positions", []) or [])
        while self._wp_pending:
            name, positions = self._wp_pending[0]
            if len(positions) != len(desired) or any(
                abs(a - d) > _WAYPOINT_FEEDBACK_TOL
                for a, d in zip(desired, positions)
            ):
                break
            self._wp_pending.pop(0)
            self._log("ok", f"{name} completado")
            siguiente = self._wp_pending[0][0] if self._wp_pending else None
            if siguiente is not None:
                self._set_status(
                    f"● ✓ {name} completado → siguiente: {siguiente}", _VERDE
                )
            else:
                self._set_status(f"● ✓ {name} completado (último punto)", _VERDE)

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
            self._log("ok", ok_text)
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
        # Si había puntos pendientes, se indica cuál no se alcanzó.
        fallo = self._wp_pending[0][0] if self._wp_pending else tag
        detalle = f"{nombre}" + (f" — {error_string}" if error_string else "")
        self._log("error", f"{fallo}: FALLÓ ({detalle})")
        self._finish_motion(
            fail_text and f"● {fail_text}" or f"● {fallo}: FALLÓ ({nombre})", _ROJO
        )

    def _finish_motion(self, status_text: str, color: str) -> None:
        """Restaura botón y sondeo de estado al terminar (o fallar) un goal."""
        self._motion_active = False
        self._motion_goal_handle = None
        self._wp_pending = None
        self._stop_requested = False
        if self._motion_button is not None:
            self._motion_button.setEnabled(True)
            self._motion_button = None
        self._set_status(status_text, color)
        self._update_point_buttons()
        self._settle_scan_verdict_after_motion(status_text)
        QTimer.singleShot(_HOME_STATUS_HOLD_MS, self._resume_status_poll)

    def _settle_scan_verdict_after_motion(self, status_text: str) -> None:
        """Cierra el veredicto del barrido cuando su goal termina o falla.

        Los botones se recalculan siempre (también tras un HOME/PUNTO/
        TRAYECTORIA o el propio IR A P1): el estado depende de la distancia
        real a P1, y eso solo lo sabe `_update_scan_buttons`. El texto del
        veredicto, en cambio, no se toca si el movimiento no era el barrido:
        sigue siendo el del último REGENERAR.
        """
        if not hasattr(self, "btn_scan_execute"):
            return
        self._update_scan_buttons()
        if not status_text.startswith("● BARRIDO"):
            return
        ok = "COMPLETADO" in status_text
        self._set_scan_verdict(
            "✓ BARRIDO COMPLETADO (escaneo P1 → P2)"
            if ok
            else f"✗ {status_text.removeprefix('● ')}",
            _VERDE if ok else _ROJO,
        )

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
            ok_text="Trayectoria completada",
            tag="TRAYECTORIA",
            fail_text="TRAYECTORIA: MOVIMIENTO FALLIDO",
            waypoints=[(p.name, list(p.q)) for p in resolved],
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