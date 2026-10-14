"""Tests de la línea verde superpuesta sobre la vista de cámara.

Se comprueba que la GUI pinta la referencia de ancho de FOV (módulo puro
`sensor_reference`) sobre la copia del frame de /camera/image_raw, sin tocar
ROS: se llama al helper de dibujo con una QImage en blanco y se inspeccionan
los píxeles. La fila donde debe caer la línea la dicta `sensor_reference`, no un
valor fijo: la referencia ya no está centrada en el eje óptico.
"""

import pytest
from PySide6.QtGui import QColor, QImage

from staubli_gui import gui as gui_mod
from staubli_gui import sensor_reference as sr


@pytest.fixture
def ventana(qapp, config_dir):
    """MainWindow con la configuración aislada y sin nodo ROS."""
    return gui_mod.MainWindow(ros_node=None)


def _imagen_blanca(width=640, height=480):
    image = QImage(width, height, QImage.Format_RGB888)
    image.fill(QColor("white"))
    return image


def _cuenta_linea_verde(image, fila):
    """Píxeles no blancos de una fila (la línea verde pintada encima)."""
    objetivo = QColor(gui_mod._VERDE).name()
    return sum(
        1
        for x in range(image.width())
        if image.pixelColor(x, fila).name() == objetivo
    )


def _fila_esperada(distance_m, height=480):
    """Fila v donde sensor_reference sitúa la línea para esa distancia."""
    (_, v0), _ = sr.working_reference_pixels(distance_m, 640, height)
    return int(round(v0))


def test_la_referencia_verde_se_pinta_sobre_el_frame(ventana):
    image = _imagen_blanca()
    distancia = gui_mod._spin_to_m(ventana._scan_distance_spin.value())
    fila = _fila_esperada(distancia, image.height())
    ventana._draw_working_reference(image)
    filas_con_linea = sum(
        _cuenta_linea_verde(image, f) for f in (fila - 1, fila, fila + 1)
    )
    assert filas_con_linea > 0
    # La referencia ocupa una franja horizontal, no toda la imagen.
    assert image.pixelColor(0, 0).name() == QColor("white").name()
    assert image.pixelColor(image.width() - 1, image.height() - 1).name() == (
        QColor("white").name()
    )


def test_la_referencia_se_recoloca_con_la_distancia(ventana):
    """Cambiar la distancia mueve la línea (v depende de la geometría real)."""
    ventana._scan_distance_spin.setValue(200)
    cerca = _imagen_blanca()
    ventana._draw_working_reference(cerca)
    fila_cerca = _fila_esperada(0.20, cerca.height())

    ventana._scan_distance_spin.setValue(500)
    lejos = _imagen_blanca()
    ventana._draw_working_reference(lejos)
    fila_lejos = _fila_esperada(0.50, lejos.height())

    assert fila_cerca != fila_lejos
    assert any(
        _cuenta_linea_verde(cerca, f) for f in (fila_cerca - 1, fila_cerca, fila_cerca + 1)
    )
    assert any(
        _cuenta_linea_verde(lejos, f) for f in (fila_lejos - 1, fila_lejos, fila_lejos + 1)
    )


def test_la_referencia_usa_la_distancia_cargada_del_perfil(ventana):
    """El valor que pinta la GUI es el del perfil de barrido cargado."""
    distancia = gui_mod._spin_to_m(ventana._scan_distance_spin.value())
    assert distancia == pytest.approx(ventana._scan_store.profile.scan_distance)


def test_sin_spin_de_distancia_no_falla():
    """El helper tolera objetos sin el control (defensivo, no rompe la GUI)."""

    class _SinSpin:
        def _draw_working_reference(self, image):
            gui_mod.MainWindow._draw_working_reference(self, image)

    image = _imagen_blanca(64, 48)
    _SinSpin()._draw_working_reference(image)  # no debe lanzar
    assert image.pixelColor(0, 0).name() == QColor("white").name()
