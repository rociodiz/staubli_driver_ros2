"""Widgets del mando virtual (gamepad) de la GUI del Staubli TX2-60L.

Este módulo NO tiene dependencias de ROS ni de cinemática: solo produce
componentes normalizados de un Twist (geométrico). El mapeo de ejes está
centralizado en una función pura ``map_twist_components`` para que sea
trivial de reasignar si la orientación resultase poco intuitiva.

Convenciones (aplicadas ya dentro de cada stick):
  - Eje x de salida: positivo -> derecha.
  - Eje y de salida: positivo -> ARRIBA (se invierte la y de pantalla).
  - Los valores se normalizan a [-1, 1] y se limitan a un círculo.

Widgets:
  - ``VirtualJoystick``: stick de un solo pulgar. Dibuja la base, la zona
    muerta (anillo discontinuo) y el pomo. devuelve al centro con una
    pequeña animación al soltar el ratón.
  - ``JoystickPanel``: caja que ensambla dos sticks (izq + der), los
    botones Z+/Z-, el control de ganancia y la lectura en vivo de los
    seis componentes (Linear/Angular). No publica nada por sí mismo.
"""

from __future__ import annotations

from typing import Tuple

from PySide6.QtCore import QPointF, QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

# ─── Paleta (misma familia que gui.py, sin importar de ahí) ────────────
_STICK_BG       = "#1a1a2e"
_STICK_BORDER   = "#34495e"
_GRIS_CLR       = "#ecf0f1"
_GRIS_MED       = "#34495e"
_BLANCO_HUESO   = "#f5f6fa"
_VERDE          = "#27ae60"
_GRIS_TXT       = "#bdc3c7"
_POMO_IZQ       = "#27ae60"
_POMO_DER       = "#f39c12"
_DEADZONE_RING  = QColor(241, 196, 15, 130)

_FAMILIA = '"DejaVu Sans", "Noto Sans", "Liberation Sans", sans-serif'
_FAM_MONO = '"DejaVu Sans Mono", "Liberation Mono", "Noto Sans Mono", monospace'

# Radio de zona muerta normalizado (fracción del radio total del stick).
_DEFAULT_DEADZONE = 0.15
# Pasos de la animación de retorno al centro al soltar (16 ms por paso).
_RETURN_STEPS = 8


# ─── Helpers geométricos ────────────────────────────────────────────────

def _clamp(v: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, v))


def _limitar_a_circulo(nx: float, ny: float) -> Tuple[float, float]:
    """Limita un vector normalizado al círculo unitario."""
    mag = (nx * nx + ny * ny) ** 0.5
    if mag > 1.0:
        return nx / mag, ny / mag
    return nx, ny


def _aplicar_deadzone(
    nx: float, ny: float, deadzone: float
) -> Tuple[float, float]:
    """Aplica la zona muerta (circular): dentro del radio -> (0, 0).

    Fuera de la zona muerta el valor NO se re-espacia: la salida queda
    "normalizada 1:1" con la posición, de modo que una deflexión del 50 %
    da 0.5 (intuitivo para escalar a velocidades reales más adelante).
    """
    mag = (nx * nx + ny * ny) ** 0.5
    if mag <= deadzone or deadzone <= 0.0:
        return 0.0, 0.0
    return nx, ny


# ─── Mapeo de ejes (función pura, fácil de reasignar) ─────────────────
def map_twist_components(
    izq_x: float,
    izq_y: float,
    der_x: float,
    der_y: float,
    z_val: float,
    gain: float = 1.0,
) -> dict:
    """Convierte los seis comandos del mando en los componentes de un Twist.

    Mapeo inicial (base_link):
        stick izq. x -> linear.x
        stick izq. y -> linear.y
        botones   Z+ / Z- -> linear.z
        stick der. x -> angular.z
        stick der. y -> angular.y
        angular.x     -> 0.0 (reservado, sin asignar)

    Todos los componentes se multiplican por ``gain``.
    """
    g = float(gain)
    return {
        "linear": {
            "x": float(izq_x) * g,
            "y": float(izq_y) * g,
            "z": float(z_val) * g,
        },
        "angular": {
            "x": 0.0,
            "y": float(der_y) * g,
            "z": float(der_x) * g,
        },
    }


# ─── Stick virtual ──────────────────────────────────────────────────────
class VirtualJoystick(QWidget):
    """Stick de un pulgar: posición circular, deadzone, retorno al centro.

    Firma: ``valueChanged(float x, float y)`` donde (x, y) ya tienen la
    deadzone aplicada y la convención "arriba = +y" (normalizados a ±1).
    """

    valueChanged = Signal(float, float)

    def __init__(
        self,
        knob_color: str = _POMO_IZQ,
        deadzone: float = _DEFAULT_DEADZONE,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._deadzone = deadzone
        self._pressed = False
        # Posición normalizada del pomo en coordenadas de pantalla (y hacia
        # abajo); almacenada para pintar. La salida invierte la y.
        self._raw = QPointF(0.0, 0.0)
        self._output = (0.0, 0.0)
        self._knob_color = QColor(knob_color)

        self._anim_timer = QTimer(self)
        self._anim_timer.setInterval(16)
        self._anim_timer.timeout.connect(self._anim_tick)
        self._anim_from = QPointF(0.0, 0.0)
        self._anim_steps = 0

        self.setMinimumSize(150, 150)
        self.setMouseTracking(True)
        self.setCursor(Qt.OpenHandCursor)

    # ── API pública ────────────────────────────────────────────────────
    @property
    def value(self) -> Tuple[float, float]:
        """Último valor de salida (deadzone aplicada, y arriba positiva)."""
        return self._output

    def set_position(self, nx: float, ny: float) -> None:
        """Fija el pomo en coords normalizadas de pantalla (y hacia abajo).

        Usado por el ratón (mouse events) y por los tests. Clampa al
        círculo y emite siempre: si el stick está quieto la salida es 0.
        """
        nx, ny = _limitar_a_circulo(_clamp(nx), _clamp(ny))
        self._raw = QPointF(nx, ny)
        self._emit_output()

    def reset(self) -> None:
        """Vuelve al centro instantáneamente (usado al desactivar joystick)."""
        self._anim_timer.stop()
        self._raw = QPointF(0.0, 0.0)
        self._emit_output()

    # ── Eventos de ratón ───────────────────────────────────────────────
    def mousePressEvent(self, event) -> None:  # noqa: N802 - firma de Qt
        if event.button() == Qt.LeftButton:
            self._pressed = True
            self._anim_timer.stop()
            self.setCursor(Qt.ClosedHandCursor)
            self._apply_event_pos(event)
            event.accept()

    def mouseMoveEvent(self, event) -> None:  # noqa: N802 - firma de Qt
        if self._pressed:
            self._apply_event_pos(event)
        event.accept()

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802 - firma de Qt
        if event.button() == Qt.LeftButton and self._pressed:
            self._pressed = False
            self.setCursor(Qt.OpenHandCursor)
            self._start_return_animation()
        event.accept()

    def _apply_event_pos(self, event) -> None:
        radio = self._radius()
        if radio <= 0:
            return
        center = self._center()
        nx = (event.position().x() - center.x()) / radio
        ny = (event.position().y() - center.y()) / radio
        self.set_position(nx, ny)

    # ── Animación de retorno al centro ─────────────────────────────────
    def _start_return_animation(self) -> None:
        self._anim_from = QPointF(self._raw)
        self._anim_steps = _RETURN_STEPS
        self._anim_timer.start()

    def _anim_tick(self) -> None:
        self._anim_steps -= 1
        t = max(0.0, self._anim_steps / _RETURN_STEPS)
        nx = self._anim_from.x() * t
        ny = self._anim_from.y() * t
        self.set_position(nx, ny)
        if self._anim_steps <= 0:
            self._anim_timer.stop()

    # ── Geometría y pintado ────────────────────────────────────────────
    def _center(self) -> QPointF:
        rect = self.rect()
        return QPointF(rect.width() / 2.0, rect.height() / 2.0)

    def _radius(self) -> float:
        return min(self.width(), self.height()) / 2.0 - 6.0

    def _emit_output(self) -> None:
        # Salida: x derecha positiva; y ARRIBA positiva (se invierte pantalla).
        ox, oy = _aplicar_deadzone(self._raw.x(), self._raw.y(), self._deadzone)
        self._output = (round(ox, 4), round(-oy, 4))
        self.update()
        self.valueChanged.emit(self._output[0], self._output[1])

    def paintEvent(self, event) -> None:  # noqa: N802 - firma de Qt
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        radio = self._radius()
        center = self._center()
        if radio <= 0:
            painter.end()
            return

        # Base: círculo oscuro con borde.
        painter.setPen(QPen(QColor(_STICK_BORDER), 2))
        painter.setBrush(QColor(_STICK_BG))
        painter.drawEllipse(
            QRectF(
                center.x() - radio,
                center.y() - radio,
                2.0 * radio,
                2.0 * radio,
            )
        )

        # Cruz central tenue.
        cross = QPen(QColor(255, 255, 255, 40), 1)
        painter.setPen(cross)
        b = radio * 0.92
        painter.drawLine(QPointF(center.x() - b, center.y()), QPointF(center.x() + b, center.y()))
        painter.drawLine(QPointF(center.x(), center.y() - b), QPointF(center.x(), center.y() + b))

        # Anillo de zona muerta (discontinuo).
        ring_pen = QPen(_DEADZONE_RING, 1, Qt.DashLine)
        painter.setPen(ring_pen)
        dzr = radio * self._deadzone
        painter.drawEllipse(
            QRectF(
                center.x() - dzr,
                center.y() - dzr,
                2.0 * dzr,
                2.0 * dzr,
            )
        )

        # Pomo.
        knob_radius = radio * 0.30
        max_off = radio - knob_radius
        kx = center.x() + self._raw.x() * max_off
        ky = center.y() + self._raw.y() * max_off
        painter.setPen(QPen(QColor(_STICK_BORDER), 2))
        painter.setBrush(self._knob_color)
        painter.drawEllipse(
            QRectF(
                kx - knob_radius,
                ky - knob_radius,
                2.0 * knob_radius,
                2.0 * knob_radius,
            )
        )
        # Luz especular pequeña en el pomo.
        painter.setBrush(QColor(255, 255, 255, 70))
        painter.setPen(Qt.NoPen)
        hr = knob_radius * 0.35
        painter.drawEllipse(
            QRectF(
                kx - 0.4 * knob_radius - hr,
                ky - 0.5 * knob_radius - hr,
                2.0 * hr,
                2.0 * hr,
            )
        )

        painter.end()


# ─── Panel del mando (gamepad) ──────────────────────────────────────────
class JoystickPanel(QGroupBox):
    """Caja "Mando virtual": sticks izq/der, Z+/Z-, ganancia y lectura.

    Señales:
        - ``toggled(bool)``: el botón ACTIVAR JOYSTICK cambió de estado.
        - ``joystick_changed()``: cualquiera de los ejes cambió (readout).
    """

    toggled = Signal(bool)
    joystick_changed = Signal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__("CONTROL MANUAL", parent)
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet(
            f"QGroupBox {{ background-color: #2c3e50; border: 1px solid #34495e;"
            f" border-radius: 6px; margin-top: 10px; padding: 8px 10px 6px 10px;"
            f" color: {_GRIS_CLR}; font-family: {_FAMILIA}; font-size: 13px;"
            f" font-weight: bold; }}"
            f"QGroupBox::title {{ subcontrol-origin: margin; left: 12px;"
            f" padding: 0 6px; }}"
            f"QGroupBox QGroupBox {{ margin-top: 6px; padding: 2px 6px 2px 6px; }}"
            f"QGroupBox QGroupBox::title {{ margin-top: 0; left: 8px;"
            f" padding: 0 4px; }}"
        )

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 6, 16, 10)
        root.setSpacing(10)
        self.setMinimumHeight(330)

        self._readout: dict[str, QLabel] = {}
        self._summary_labels: dict[str, QLabel] = {}

        # ── Fila superior: sticks grandes + TRASLACIÓN / ROTACIÓN ─────
        row_sticks = QHBoxLayout()
        row_sticks.setSpacing(44)

        self.stick_izq = VirtualJoystick(knob_color=_POMO_IZQ)
        col_izq = QVBoxLayout()
        col_izq.setSpacing(6)
        col_izq.addWidget(self._stick_title("STICK IZQ."), 0, Qt.AlignHCenter)
        col_izq.addWidget(self.stick_izq, 0, Qt.AlignHCenter)
        col_izq.addWidget(
            self._subsection(
                "TRASLACIÓN",
                [
                    ("X", self._make_readout_value("lin_x")),
                    ("Y", self._make_readout_value("lin_y")),
                ],
            ),
            0,
            Qt.AlignHCenter,
        )

        self.stick_der = VirtualJoystick(knob_color=_POMO_DER)
        col_der = QVBoxLayout()
        col_der.setSpacing(6)
        col_der.addWidget(self._stick_title("STICK DER."), 0, Qt.AlignHCenter)
        col_der.addWidget(self.stick_der, 0, Qt.AlignHCenter)
        col_der.addWidget(
            self._subsection(
                "ROTACIÓN",
                [
                    ("X (reservado)", self._make_readout_value("ang_x")),
                    ("Y", self._make_readout_value("ang_y")),
                    ("Z", self._make_readout_value("ang_z")),
                ],
            ),
            0,
            Qt.AlignHCenter,
        )

        row_sticks.addStretch()
        row_sticks.addLayout(col_izq)
        row_sticks.addStretch()
        row_sticks.addLayout(col_der)
        row_sticks.addStretch()
        root.addLayout(row_sticks)

        # ── Fila de control: Z+/Z- (+ valor de Z), ACTIVAR, GANANCIA ──
        row_ctrl = QHBoxLayout()
        row_ctrl.setSpacing(16)

        self.btn_z_plus = self._make_z_button("Z+")
        self.btn_z_minus = self._make_z_button("Z-")
        z_pair = QHBoxLayout()
        z_pair.setSpacing(12)
        z_pair.addWidget(self.btn_z_plus)
        z_pair.addWidget(self.btn_z_minus)
        z_pair.addSpacing(4)
        z_lbl = QLabel("Z")
        z_lbl.setStyleSheet(
            f"color: {_GRIS_TXT}; font-family: {_FAM_MONO}; font-size: 12px;"
        )
        z_pair.addWidget(z_lbl)
        z_pair.addWidget(self._make_readout_value("lin_z"))
        row_ctrl.addLayout(z_pair)

        row_ctrl.addStretch()

        self.btn_toggle = QPushButton("JOYSTICK DESACTIVADO")
        self.btn_toggle.setCheckable(True)
        self.btn_toggle.setMinimumWidth(230)
        self.btn_toggle.setStyleSheet(
            f"QPushButton {{ background-color: {_VERDE}; color: {_BLANCO_HUESO};"
            f" border: none; border-radius: 5px; padding: 8px 16px;"
            f" font-family: {_FAMILIA}; font-size: 13px; font-weight: bold;"
            f" min-height: 36px; }}"
        )
        self.btn_toggle.setCursor(Qt.PointingHandCursor)
        self.btn_toggle.toggled.connect(self._on_toggle)
        row_ctrl.addWidget(self.btn_toggle)

        row_ctrl.addStretch()

        gain_lbl = QLabel("GANANCIA")
        gain_lbl.setStyleSheet(
            f"color: {_GRIS_CLR}; font-family: {_FAMILIA}; font-size: 11px;"
            f" font-weight: bold;"
        )
        gain_lbl.setAlignment(Qt.AlignHCenter)
        self._gain = QDoubleSpinBox()
        self._gain.setRange(0.10, 2.00)
        self._gain.setSingleStep(0.05)
        self._gain.setValue(1.00)
        self._gain.setDecimals(2)
        self._gain.setAlignment(Qt.AlignCenter)
        self._gain.setStyleSheet(
            f"QDoubleSpinBox {{ background-color: {_STICK_BG}; color: {_BLANCO_HUESO};"
            f" border: 1px solid {_STICK_BORDER}; border-radius: 4px;"
            f" padding: 4px 8px; font-family: {_FAM_MONO}; font-size: 13px; }}"
        )
        self._gain.setMinimumWidth(72)
        self._gain.valueChanged.connect(self._on_change)
        col_gain = QVBoxLayout()
        col_gain.setSpacing(4)
        col_gain.addWidget(gain_lbl)
        col_gain.addWidget(self._gain)
        row_ctrl.addLayout(col_gain)

        root.addLayout(row_ctrl)

        # ── Resumen VALORES ACTUALES (Twist completo, separado de los
        #    sticks para evitar solapes) ────────────────────────────────
        sum_row = QHBoxLayout()
        sum_row.setSpacing(16)
        sum_head = QLabel("VALORES ACTUALES")
        sum_head.setStyleSheet(
            f"color: {_GRIS_CLR}; font-family: {_FAMILIA}; font-size: 11px;"
            f" font-weight: bold;"
        )
        sum_row.addWidget(sum_head)
        for eje, block in (
            ("LIN", (("lin_x", "X"), ("lin_y", "Y"), ("lin_z", "Z"))),
            ("ANG", (("ang_x", "X"), ("ang_y", "Y"), ("ang_z", "Z"))),
        ):
            parte = QHBoxLayout()
            parte.setSpacing(6)
            cab = QLabel(eje)
            cab.setStyleSheet(
                f"color: {_GRIS_TXT}; font-family: {_FAM_MONO}; font-size: 11px;"
            )
            parte.addWidget(cab)
            for key, label_eje in block:
                lbl = QLabel(label_eje)
                lbl.setStyleSheet(
                    f"color: {_GRIS_TXT}; font-family: {_FAM_MONO}; font-size: 11px;"
                )
                parte.addWidget(lbl)
                parte.addWidget(self._make_summary_value(key))
            sum_row.addLayout(parte)
        sum_row.addStretch()
        root.addLayout(sum_row)

        # ── Estado interno de los botones Z ────────────────────────
        self._zp = False
        self._zm = False

        # Conexiones: cualquier cambio -> readout + señal.
        self.stick_izq.valueChanged.connect(lambda *_: self._on_change())
        self.stick_der.valueChanged.connect(lambda *_: self._on_change())

    # ── Helpers de construcción ────────────────────────────────────────
    def _stick_title(self, texto: str) -> QLabel:
        label = QLabel(texto)
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet(
            f"color: {_GRIS_CLR}; font-family: {_FAMILIA}; font-size: 12px;"
            f" font-weight: bold;"
        )
        return label

    def _make_readout_value(self, key: str) -> QLabel:
        """Crea una etiqueta de valor (registrada en self._readout)."""
        valor = QLabel("0.00")
        valor.setStyleSheet(
            f"color: {_GRIS_CLR}; font-family: {_FAM_MONO};"
            f" font-size: 12px; background-color: {_STICK_BG};"
            f" border: 1px solid {_STICK_BORDER}; border-radius: 3px;"
            f" padding: 1px 8px; min-width: 46px; min-height: 18px;"
        )
        valor.setAlignment(Qt.AlignRight)
        self._readout[key] = valor
        return valor

    def _make_summary_value(self, key: str) -> QLabel:
        """Etiqueta del resumen inferior "VALORES ACTUALES" (no en _readout)."""
        valor = QLabel("0.00")
        valor.setStyleSheet(
            f"color: {_GRIS_CLR}; font-family: {_FAM_MONO};"
            f" font-size: 12px; background-color: {_STICK_BG};"
            f" border: 1px solid {_STICK_BORDER}; border-radius: 3px;"
            f" padding: 1px 6px; min-width: 44px; min-height: 16px;"
        )
        valor.setAlignment(Qt.AlignRight)
        self._summary_labels[key] = valor
        return valor

    def _subsection(self, titulo: str, pares) -> QGroupBox:
        """Bloque titulado (TRASLACIÓN/ROTACIÓN) con pares eje+valor."""
        box = QGroupBox(titulo)
        box.setMinimumWidth(150)
        grid = QGridLayout(box)
        grid.setContentsMargins(8, 2, 8, 4)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(3)
        for row, (eje, valor) in enumerate(pares):
            lbl = QLabel(eje)
            lbl.setStyleSheet(
                f"color: {_GRIS_TXT}; font-family: {_FAM_MONO}; font-size: 12px;"
            )
            grid.addWidget(lbl, row, 0, Qt.AlignRight | Qt.AlignVCenter)
            grid.addWidget(valor, row, 1)
        return box

    # ── Helpers de construcción ────────────────────────────────────────
    def _make_z_button(self, texto: str) -> QPushButton:
        btn = QPushButton(texto)
        btn.setStyleSheet(
            f"QPushButton {{ background-color: #2980b9; color: {_BLANCO_HUESO};"
            f" border: none; border-radius: 5px; padding: 6px 16px;"
            f" font-family: {_FAMILIA}; font-size: 13px; font-weight: bold;"
            f" min-height: 30px; }}"
            f"QPushButton:pressed {{ background-color: #1f618d; }}"
        )
        btn.setCursor(Qt.PointingHandCursor)
        btn.pressed.connect(lambda: self._set_z_pressed(texto == "Z+", True))
        btn.released.connect(lambda: self._set_z_pressed(texto == "Z+", False))
        return btn

    # ── Estado y señales ──────────────────────────────────────────────
    def _set_z_pressed(self, plus: bool, pressed: bool) -> None:
        if plus:
            self._zp = pressed
        else:
            self._zm = pressed
        self._on_change()

    def _on_toggle(self, active: bool) -> None:
        self.btn_toggle.setText(
            "JOYSTICK ACTIVADO" if active else "JOYSTICK DESACTIVADO"
        )
        self.toggled.emit(active)

    def _on_change(self, *_args) -> None:
        self._update_readout()
        self.joystick_changed.emit()

    def z_value(self) -> float:
        """+1 / -1 / 0 según los botones Z+/Z- pulsados."""
        return (1.0 if self._zp else 0.0) - (1.0 if self._zm else 0.0)

    def twist_components(self) -> dict:
        """Componentes del Twist (normalizados, con ganancia aplicada)."""
        izq = self.stick_izq.value
        der = self.stick_der.value
        return map_twist_components(
            izq_x=izq[0],
            izq_y=izq[1],
            der_x=der[0],
            der_y=der[1],
            z_val=self.z_value(),
            gain=self._gain.value(),
        )

    def reset(self) -> None:
        """Centra los sticks y libera Z+/Z- (sin tocar el toggle)."""
        self.stick_izq.reset()
        self.stick_der.reset()
        self._zp = False
        self._zm = False
        self._update_readout()
        self.joystick_changed.emit()

    def set_gain(self, value: float) -> None:
        self._gain.setValue(value)

    def gain(self) -> float:
        return self._gain.value()

    # ── Readout ───────────────────────────────────────────────────────
    def _update_readout(self) -> None:
        comp = self.twist_components()
        mapa = {
            "lin_x": comp["linear"]["x"],
            "lin_y": comp["linear"]["y"],
            "lin_z": comp["linear"]["z"],
            "ang_x": comp["angular"]["x"],
            "ang_y": comp["angular"]["y"],
            "ang_z": comp["angular"]["z"],
        }
        for key, valor in mapa.items():
            label = self._readout.get(key)
            if label is not None:
                label.setText(f"{valor:.2f}")
            summary = self._summary_labels.get(key)
            if summary is not None:
                summary.setText(f"{valor:.2f}")