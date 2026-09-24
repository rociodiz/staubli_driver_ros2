"""Controles FRAME del control manual de la GUI del Staubli TX2-60L.

Este módulo NO tiene dependencias de ROS ni de cinemática: solo produce
componentes normalizados de un Twist (geométrico). El control manual usa
seis pares de botones +/−, uno por cada grado de libertad, con el mapeo
FRAME del mando del Staubli:

    X  -> Twist.linear.x      RX -> Twist.angular.x
    Y  -> Twist.linear.y      RY -> Twist.angular.y
    Z  -> Twist.linear.z      RZ -> Twist.angular.z

El mapeo está centralizado en la función pura ``map_twist_components`` para
que sea trivial de reasignar si se quisiera otra convención. Cada botón
produce velocidad (+ positiva, − negativa) mientras está pulsado y se
detiene al soltarlo; ``FrameControlPanel`` no publica nada por sí mismo: la
GUI publica /cmd_vel a partir de ``twist_components()``.

Widgets:
  - ``FrameControlPanel``: caja que ensambla los seis pares +/−, la
    velocidad (0-100 %), la ganancia, la lectura en vivo de los seis
    componentes (Linear/Angular) y el indicador MODO ACTUAL
    (trayectoria/manual). No publica nada por sí mismo: la GUI decide
    cuándo y cómo cambiar de controlador.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

# ─── Paleta (misma familia que gui.py, sin importar de ahí) ────────────
_BG            = "#1a1a2e"
_BORDER        = "#34495e"
_GRIS_CLR      = "#ecf0f1"
_GRIS_MED      = "#34495e"
_BLANCO_HUESO  = "#f5f6fa"
_VERDE         = "#27ae60"
_NARANJA       = "#f39c12"
_ROJO          = "#e74c3c"
_GRIS_TXT      = "#bdc3c7"

_FAMILIA = '"DejaVu Sans", "Noto Sans", "Liberation Sans", sans-serif'
_FAM_MONO = '"DejaVu Sans Mono", "Liberation Mono", "Noto Sans Mono", monospace'

# Orden y etiquetas de los seis grados de libertad (mapeo FRAME).
_FRAME_AXES = (
    ("X", "lin_x"),
    ("Y", "lin_y"),
    ("Z", "lin_z"),
    ("RX", "ang_x"),
    ("RY", "ang_y"),
    ("RZ", "ang_z"),
)


# ─── Mapeo de ejes (función pura, fácil de reasignar) ─────────────────
def map_twist_components(
    x_plus: bool = False,
    x_minus: bool = False,
    y_plus: bool = False,
    y_minus: bool = False,
    z_plus: bool = False,
    z_minus: bool = False,
    rx_plus: bool = False,
    rx_minus: bool = False,
    ry_plus: bool = False,
    ry_minus: bool = False,
    rz_plus: bool = False,
    rz_minus: bool = False,
    gain: float = 1.0,
    velocity_scale: float = 1.0,
) -> dict:
    """Convierte los seis grados de libertad del control FRAME en un Twist.

    Mapeo solicitado:
        X  -> Twist.linear.x    RX -> Twist.angular.x
        Y  -> Twist.linear.y    RY -> Twist.angular.y
        Z  -> Twist.linear.z    RZ -> Twist.angular.z

    + produce velocidad positiva y − velocidad negativa; si nada está
    pulsado el componente es 0. Cada componente se multiplica por ``gain``
    y por ``velocity_scale`` (fracción 0..1 del control VELOCIDAD).
    """
    g = float(gain) * float(velocity_scale)

    def _vel(plus: bool, minus: bool) -> float:
        if plus and minus:
            return 0.0
        if plus:
            return 1.0 * g
        if minus:
            return -1.0 * g
        return 0.0

    return {
        "linear": {
            "x": _vel(x_plus, x_minus),
            "y": _vel(y_plus, y_minus),
            "z": _vel(z_plus, z_minus),
        },
        "angular": {
            "x": _vel(rx_plus, rx_minus),
            "y": _vel(ry_plus, ry_minus),
            "z": _vel(rz_plus, rz_minus),
        },
    }


# ─── Panel de control manual (FRAME) ────────────────────────────────────
class FrameControlPanel(QGroupBox):
    """Caja "CONTROL MANUAL": seis pares de botones +/− (X..RZ).

    Señales:
        - ``toggled(bool)``: el botón ACTIVAR MANUAL fue pulsado (estado
          deseado). La GUI no lo toma como hecho real: gestiona el cambio de
          controlador y luego refleja el resultado con ``set_toggle_state``.
        - ``manual_changed()``: cualquiera de los botones o la velocidad
          cambió (readout).

    El botón de activar es ``btn_toggle``. Su texto lo gestiona la GUI
    (``set_toggle_busy`` / ``set_toggle_state``), no el propio widget.
    """

    toggled = Signal(bool)
    manual_changed = Signal()

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

        # Estado pulsado de cada grado de libertad: {"plus": bool, "minus": bool}.
        self._state: dict[str, dict[str, bool]] = {
            axis: {"plus": False, "minus": False} for axis, _ in _FRAME_AXES
        }
        self._plus_buttons: dict[str, QPushButton] = {}
        self._minus_buttons: dict[str, QPushButton] = {}
        self._readout: dict[str, QLabel] = {}
        self._summary_labels: dict[str, QLabel] = {}

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 6, 16, 10)
        root.setSpacing(10)
        self.setMinimumHeight(330)

        # ── Botones +/− por grado de libertad (TRASLACIÓN / ROTACIÓN) ──
        row_axes = QHBoxLayout()
        row_axes.setSpacing(24)
        row_axes.addStretch()
        row_axes.addWidget(self._axes_box("TRASLACIÓN", ("X", "Y", "Z")))
        row_axes.addStretch()
        row_axes.addWidget(self._axes_box("ROTACIÓN", ("RX", "RY", "RZ")))
        row_axes.addStretch()
        root.addLayout(row_axes)

        # ── Fila de control: ACTIVAR MANUAL, GANANCIA y VELOCIDAD ──────
        row_ctrl = QHBoxLayout()
        row_ctrl.setSpacing(16)

        self.btn_toggle = QPushButton("ACTIVAR MANUAL")
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
            f"QDoubleSpinBox {{ background-color: {_BG}; color: {_BLANCO_HUESO};"
            f" border: 1px solid {_BORDER}; border-radius: 4px;"
            f" padding: 4px 8px; font-family: {_FAM_MONO}; font-size: 13px; }}"
        )
        self._gain.setMinimumWidth(72)
        self._gain.valueChanged.connect(self._on_change)
        col_gain = QVBoxLayout()
        col_gain.setSpacing(4)
        col_gain.addWidget(gain_lbl)
        col_gain.addWidget(self._gain)
        row_ctrl.addLayout(col_gain)

        # Slider VELOCIDAD (0-100 %) independiente de GANANCIA.
        vel_lbl = QLabel("VELOCIDAD")
        vel_lbl.setStyleSheet(
            f"color: {_GRIS_CLR}; font-family: {_FAMILIA}; font-size: 11px;"
            f" font-weight: bold;"
        )
        vel_lbl.setAlignment(Qt.AlignHCenter)
        self._vel = QSlider(Qt.Horizontal)
        self._vel.setRange(0, 100)
        self._vel.setValue(50)
        self._vel.setSingleStep(5)
        self._vel.setMinimumWidth(140)
        self._vel.setStyleSheet(
            f"QSlider::groove:horizontal {{ height: 6px; background: {_BG};"
            f" border: 1px solid {_BORDER}; border-radius: 3px; }}"
            f"QSlider::handle:horizontal {{ background: {_NARANJA}; width: 16px;"
            f" height: 16px; margin: -6px 0; border-radius: 8px; }}"
        )
        self._vel_pct = QLabel("50 %")
        self._vel_pct.setStyleSheet(
            f"color: {_BLANCO_HUESO}; font-family: {_FAM_MONO}; font-size: 13px;"
            f" background-color: {_BG}; border: 1px solid {_BORDER};"
            f" border-radius: 4px; padding: 4px 8px;"
        )
        self._vel_pct.setAlignment(Qt.AlignCenter)
        self._vel_pct.setMinimumWidth(58)
        self._vel.valueChanged.connect(self._on_vel_changed)
        row_vel = QHBoxLayout()
        row_vel.setSpacing(8)
        row_vel.addWidget(self._vel)
        row_vel.addWidget(self._vel_pct)
        col_vel = QVBoxLayout()
        col_vel.setSpacing(4)
        col_vel.addWidget(vel_lbl)
        col_vel.addLayout(row_vel)
        row_ctrl.addLayout(col_vel)

        root.addLayout(row_ctrl)

        # ── Indicador MODO ACTUAL (modos excluyentes) ───────────────────
        mode_row = QHBoxLayout()
        mode_row.setSpacing(10)
        self._mode_dot = QLabel("●")
        self._mode_dot.setStyleSheet(
            f"color: {_GRIS_MED}; font-family: {_FAM_MONO}; font-size: 22px;"
        )
        mode_row.addWidget(self._mode_dot)
        col_modo = QVBoxLayout()
        col_modo.setSpacing(2)
        self._mode_label = QLabel("MODO DESCONOCIDO")
        self._mode_label.setStyleSheet(
            f"color: {_BLANCO_HUESO}; font-family: {_FAMILIA};"
            f" font-size: 13px; font-weight: bold;"
        )
        self._mode_info = QLabel("esperando información de controller_manager…")
        self._mode_info.setStyleSheet(
            f"color: {_GRIS_TXT}; font-family: {_FAM_MONO}; font-size: 11px;"
        )
        col_modo.addWidget(self._mode_label)
        col_modo.addWidget(self._mode_info)
        mode_row.addLayout(col_modo)
        mode_row.addStretch()
        root.addLayout(mode_row)

        # ── Resumen VALORES ACTUALES (Twist completo, separado de los
        #    botones para evitar solapes) ────────────────────────────────
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

    # ── Construcción de los bloques de botones ─────────────────────────
    def _axes_box(self, titulo: str, axes: tuple[str, ...]) -> QGroupBox:
        """Bloque titulado (TRASLACIÓN/ROTACIÓN) con pares +/− por eje."""
        box = QGroupBox(titulo)
        box.setMinimumWidth(230)
        grid = QGridLayout(box)
        grid.setContentsMargins(10, 4, 10, 6)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(6)

        for row, axis in enumerate(axes):
            key = {"X": "lin_x", "Y": "lin_y", "Z": "lin_z",
                   "RX": "ang_x", "RY": "ang_y", "RZ": "ang_z"}[axis]
            lbl = QLabel(axis)
            lbl.setStyleSheet(
                f"color: {_GRIS_TXT}; font-family: {_FAM_MONO}; font-size: 14px;"
                f" font-weight: bold;"
            )
            grid.addWidget(lbl, row, 0, Qt.AlignRight | Qt.AlignVCenter)

            btn_minus = self._make_axis_button(axis, "minus", "−")
            btn_plus = self._make_axis_button(axis, "plus", "+")
            self._minus_buttons[axis] = btn_minus
            self._plus_buttons[axis] = btn_plus
            grid.addWidget(btn_minus, row, 1)
            grid.addWidget(self._make_readout_value(key), row, 2)
            grid.addWidget(btn_plus, row, 3)
        return box

    def _make_axis_button(self, axis: str, sign: str, texto: str) -> QPushButton:
        estado = self._state[axis]
        btn = QPushButton(texto)
        color = _VERDE if sign == "plus" else _ROJO
        btn.setStyleSheet(
            f"QPushButton {{ background-color: {_GRIS_MED}; color: {_BLANCO_HUESO};"
            f" border: none; border-radius: 5px;"
            f" font-family: {_FAM_MONO}; font-size: 16px; font-weight: bold;"
            f" min-width: 40px; min-height: 30px; }}"
            f"QPushButton:pressed {{ background-color: {color}; }}"
        )
        btn.setCursor(Qt.PointingHandCursor)

        def _set(pressed: bool) -> None:
            if sign == "plus":
                estado["plus"] = pressed
            else:
                estado["minus"] = pressed
            self._on_change()

        btn.pressed.connect(lambda: _set(True))
        btn.released.connect(lambda: _set(False))
        return btn

    def _make_readout_value(self, key: str) -> QLabel:
        """Crea una etiqueta de valor (registrada en self._readout)."""
        valor = QLabel("0.00")
        valor.setStyleSheet(
            f"color: {_GRIS_CLR}; font-family: {_FAM_MONO};"
            f" font-size: 12px; background-color: {_BG};"
            f" border: 1px solid {_BORDER}; border-radius: 3px;"
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
            f" font-size: 12px; background-color: {_BG};"
            f" border: 1px solid {_BORDER}; border-radius: 3px;"
            f" padding: 1px 6px; min-width: 44px; min-height: 16px;"
        )
        valor.setAlignment(Qt.AlignRight)
        self._summary_labels[key] = valor
        return valor

    # ── Estado y señales ──────────────────────────────────────────────
    def _on_toggle(self, active: bool) -> None:
        # El texto/visual del botón lo fija la GUI con set_toggle_state;
        # aquí solo se reenvía la intención (fallible).
        self.toggled.emit(active)

    def _on_change(self, *_args) -> None:
        self._update_readout()
        self.manual_changed.emit()

    def _on_vel_changed(self, value: int) -> None:
        self._vel_pct.setText(f"{value} %")
        self._on_change()

    def set_button_pressed(self, axis: str, sign: str, pressed: bool) -> None:
        """Puede usar para tests: simula pulsar/soltar un botón +/−."""
        if axis not in self._state or sign not in ("plus", "minus"):
            return
        self._state[axis][sign] = bool(pressed)
        self._on_change()

    def twist_components(self) -> dict:
        """Componentes del Twist (normalizados, con ganancia aplicada)."""
        est = self._state
        return map_twist_components(
            x_plus=est["X"]["plus"], x_minus=est["X"]["minus"],
            y_plus=est["Y"]["plus"], y_minus=est["Y"]["minus"],
            z_plus=est["Z"]["plus"], z_minus=est["Z"]["minus"],
            rx_plus=est["RX"]["plus"], rx_minus=est["RX"]["minus"],
            ry_plus=est["RY"]["plus"], ry_minus=est["RY"]["minus"],
            rz_plus=est["RZ"]["plus"], rz_minus=est["RZ"]["minus"],
            gain=self._gain.value(),
            velocity_scale=self.velocity_percent() / 100.0,
        )

    def reset(self) -> None:
        """Suelta todos los botones +/− (sin tocar el toggle)."""
        for axis in self._state:
            self._state[axis]["plus"] = False
            self._state[axis]["minus"] = False
        self._update_readout()
        self.manual_changed.emit()

    def set_gain(self, value: float) -> None:
        self._gain.setValue(value)

    def gain(self) -> float:
        return self._gain.value()

    def set_velocity_percent(self, value: int) -> None:
        self._vel.setValue(int(value))

    def velocity_percent(self) -> int:
        return self._vel.value()

    # ── Modo actual (indicador) y estado del botón ─────────────────────
    def set_mode_state(
        self,
        mode: str,
        controller: str | None = None,
        velocity_pct: int | None = None,
    ) -> None:
        """Actualiza el indicador MODO ACTUAL.

        mode: "trayectoria" | "manual" | "seguro" | "conflicto" |
              "desconocido".
        controller: nombre del controlador activo (texto informativo).
        velocity_pct: velocidad actual en % (texto informativo).
        """
        estilos = {
            "trayectoria": (_VERDE, "MODO TRAYECTORIA"),
            "manual": (_NARANJA, "MODO MANUAL (CONTROL FRAME)"),
            "seguro": (_GRIS_MED, "MODO SEGURO (sin control de movimiento)"),
            "conflicto": ("#e74c3c", "CONFLICTO DE CONTROLADORES"),
            "desconocido": (_GRIS_MED, "MODO DESCONOCIDO"),
        }
        color, texto = estilos.get(mode, (_GRIS_MED, "MODO DESCONOCIDO"))
        self._mode_dot.setStyleSheet(
            f"color: {color}; font-family: {_FAM_MONO}; font-size: 22px;"
        )
        self._mode_label.setText(texto)
        pct = velocity_pct if velocity_pct is not None else "-"
        if mode == "trayectoria":
            self._mode_info.setText(
                f"controlador: {controller or 'joint_trajectory_controller'}"
                f" · velocidad {pct} %"
            )
        elif mode == "manual":
            self._mode_info.setText(
                f"controlador: {controller or 'forward_velocity_controller'}"
                f" · velocidad {pct} %"
            )
        elif mode == "conflicto":
            self._mode_info.setText(
                "joint_trajectory_controller y forward_velocity_controller"
                " activos a la vez: se volverá a modo trayectoria"
            )
        elif mode == "seguro":
            self._mode_info.setText(
                "mando detenido; active un modo para mover el robot"
            )
        else:
            self._mode_info.setText(
                "esperando información de controller_manager…"
            )

    def set_toggle_busy(self) -> None:
        """Bloquea el botón mientras se realiza el cambio de controlador."""
        self.btn_toggle.blockSignals(True)
        self.btn_toggle.setEnabled(False)
        self.btn_toggle.setText("CAMBIO DE CONTROLADOR…")
        self.btn_toggle.blockSignals(False)

    def set_toggle_state(self, active: bool) -> None:
        """Refleja el estado real del control manual en el botón."""
        self.btn_toggle.blockSignals(True)
        self.btn_toggle.setChecked(bool(active))
        self.btn_toggle.setText(
            "VOLVER A TRAYECTORIA" if active else "ACTIVAR MANUAL"
        )
        self.btn_toggle.setEnabled(True)
        self.btn_toggle.blockSignals(False)

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