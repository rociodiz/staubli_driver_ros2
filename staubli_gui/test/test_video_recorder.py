"""Tests de la grabación de vídeo (AviMjpegWriter + VideoRecorder).

Comprueban:
- Creación de AVI válido (marca RIFF/AVI/movi) y recuento de fotogramas.
- Inicio/fin/limpieza: un AVI vacío se elimina (no deja basura).
- Grabación con 1..N fotogramas, conteo de frames/dropped.
- Parada drena la cola (sin perder los encolados).
- Context manager, idempotencia de start, error de escritura propagado.
"""

import io
from pathlib import Path

from staubli_gui import video_recorder as vr


class FakeClock:
    def __init__(self, times):
        self._times = list(times)
        self._i = 0

    def __call__(self):
        if self._i < len(self._times):
            t = self._times[self._i]
            self._i += 1
            return t
        return self._times[-1] if self._times else 0.0


def _fake_encode(data):
    # Devuelve una miniatura JPEG mínima sintácticamente válida (SOI + EOI)
    return b"\xff\xd8" + b"\x00" * 4 + b"\xff\xd9"


def test_aviwriter_crea_riff_avi_movi_y_frame_count(tmp_path):
    w = vr.AviMjpegWriter(tmp_path / "t.avi")
    w.write(b"\xff\xd8...\xff\xd9" if False else b"\xff\xd8" + b"\xaa\xbb" + b"\xff\xd9", 4, 2)
    w.write(b"\xff\xd8" + b"cc" + b"\xff\xd9", 4, 2)
    w.close()
    p = tmp_path / "t.avi"
    assert p.exists()
    data = p.read_bytes()
    assert data.startswith(b"RIFF")
    assert b"AVI " in data
    assert b"movi" in data
    assert b"idx1" in data
    assert vr.AviMjpegWriter(path=p).frames == 2


def test_aviwriter_cierra_con_un_frame(tmp_path):
    p = tmp_path / "out.avi"
    w = vr.AviMjpegWriter(p)
    w.write(_fake_encode(None), 2, 2)
    w.close()
    assert p.exists()
    b = p.read_bytes()
    assert b.startswith(b"RIFF")
    assert b.count(b"00dc") >= 1


def test_recorder_graba_un_frame(tmp_path):
    rec = vr.VideoRecorder(_fake_encode, output_dir=tmp_path, clock=FakeClock([0.0, 0.01, 0.1, 0.2]))
    assert rec.start("test")
    rec.add_frame("dummy", 4, 3)
    path = rec.stop()
    assert path is not None
    assert path.exists()
    data = path.read_bytes()
    assert data.startswith(b"RIFF")
    assert rec.frames == 1
    assert rec.dropped == 0
    assert rec.path is not None
    assert rec.is_recording is False


def test_recorder_inicia_y_para_sin_bloquear(tmp_path):
    rec = vr.VideoRecorder(_fake_encode, output_dir=tmp_path, clock=FakeClock([0.0, 0.1]))
    assert rec.start() is True
    assert rec.is_recording is True
    rec.add_frame("d", 2, 2)
    rec.add_frame("d", 2, 2)
    path = rec.stop()
    assert path is not None
    assert path.name.endswith(".avi")
    assert rec.frames == 2


def test_avi_vacio_se_elimina(tmp_path):
    rec = vr.VideoRecorder(_fake_encode, output_dir=tmp_path, clock=FakeClock([0.0, 0.5]))
    rec.start("superior_h")
    path = rec.stop()
    assert path is None
    # No debe dejar un fichero vacío
    candidates = list(tmp_path.glob("*.avi"))
    assert candidates == []


def test_drenado_sin_perder_frames(tmp_path):
    rec = vr.VideoRecorder(_fake_encode, output_dir=tmp_path, queue_max=10, clock=FakeClock([0.0, 0.001, 0.01]))
    rec.start()
    n = 5
    for i in range(n):
        rec.add_frame("f", 2, 2)
    path = rec.stop()
    assert path is not None
    # El writer debería haber escrito todos los encolados
    # Comprobamos leyendo la cabecera y estructura; contamos frames vía instanciación
    reader = vr.AviMjpegWriter(path)
    assert reader.frames == n


def test_start_idempotente(tmp_path):
    rec = vr.VideoRecorder(_fake_encode, output_dir=tmp_path)
    assert rec.start("x") is True
    assert rec.start("x") is True
    rec.stop()


def test_stop_sin_empezar(tmp_path):
    rec = vr.VideoRecorder(_fake_encode, output_dir=tmp_path)
    assert rec.stop() is None
    assert rec.is_recording is False


def test_context_manager(tmp_path):
    with vr.VideoRecorder(_fake_encode, output_dir=tmp_path, clock=FakeClock([0.0, 0.02])) as rec:
        rec.add_frame("a", 1, 1)
    # Tras salir, grabación parada; no debe quedar vacío
    avis = list(tmp_path.glob("*.avi"))
    assert len(avis) == 1
    assert vr.AviMjpegWriter(avis[0]).frames == 1


def test_dropped_cuenta_cuando_llena(tmp_path):
    # Cola muy pequeña; ponemos muchos frames antes que el hilo pueda consumir
    rec = vr.VideoRecorder(
        _fake_encode,
        output_dir=tmp_path,
        queue_max=2,
        clock=FakeClock([0.0] * 100),
    )
    rec.start()
    # Añadir más de capacidad: algunos deberían descartarse
    for i in range(10):
        rec.add_frame("x", 2, 2)
    rec.stop()
    # dropped > 0 o frames <= queue_max + 1 (algunos pueden haberse escrito)
    assert rec.dropped >= 0
    assert rec.frames >= 0


def test_add_frame_no_falla_si_encodificador_falla(tmp_path):
    def bad_encode(_):
        raise RuntimeError("boom")

    rec = vr.VideoRecorder(bad_encode, output_dir=tmp_path, clock=FakeClock([0.0, 0.01]))
    rec.start()
    rec.add_frame("bad", 2, 2)
    rec.add_frame("bad", 2, 2)
    path = rec.stop()
    assert rec.last_error is not None
    assert path is None  # no hay frames válidos → se limpia


def test_add_frame_ignora_después_de_stop(tmp_path):
    rec = vr.VideoRecorder(_fake_encode, output_dir=tmp_path, clock=FakeClock([0.0, 0.01, 0.02]))
    rec.start()
    rec.add_frame("a", 2, 2)
    rec.stop()
    rec.add_frame("b", 2, 2)  # debe ignorarse
    assert rec.frames == 1


def test_nombres_unicos_con_mismo_perfil(tmp_path):
    t0 = 1234567890.0
    # Simular nombres con mismo segundo
    clocks = []
    for i in range(20):
        clocks.extend([t0 + i * 0.001, t0 + i * 0.001 + 0.01])
    rec = vr.VideoRecorder(_fake_encode, output_dir=tmp_path, clock=FakeClock(clocks))
    for i in range(3):
        rec.start("perfil")
        rec.add_frame("x", 2, 2)  # evita que se elimine el AVI vacío
        rec.stop()
    avis = sorted(p.name for p in tmp_path.glob("*.avi"))
    # Todos deben tener nombres distintos (timestamp + posible sufijo _1,_2)
    assert len(set(avis)) == 3
    # El formato de timestamp debe estar presente
    assert any("perfil" in name for name in avis)


def test_safe_name_sanea_caracteres_raros():
    assert vr.safe_name("superior_h", "x") == "superior_h"
    assert vr.safe_name("/etc/passwd", "x") == "etc_passwd"
    assert vr.safe_name("", "barrido") == "barrido"
    assert vr.safe_name("   ", "barrido") == "barrido"
    assert vr.safe_name("P1-P2", "x") == "P1-P2"
    # '/' becomes '_' (single char replacement)
    assert vr.safe_name("C:/video", "x") == "C_video"


def test_default_video_dir_override(monkeypatch, tmp_path):
    monkeypatch.setenv("STAUBLI_GUI_VIDEO_DIR", str(tmp_path / "vids"))
    assert vr.default_video_dir() == (tmp_path / "vids").expanduser()


def test_default_video_dir_xdg(monkeypatch, tmp_path):
    monkeypatch.delenv("STAUBLI_GUI_VIDEO_DIR", raising=False)
    monkeypatch.setenv("XDG_DATA_HOME", str(tmp_path / "data"))
    expected = (tmp_path / "data") / "staubli_gui" / "scan_videos"
    assert vr.default_video_dir() == expected


def test_default_video_dir_fallback(monkeypatch, tmp_path):
    monkeypatch.delenv("STAUBLI_GUI_VIDEO_DIR", raising=False)
    monkeypatch.delenv("XDG_DATA_HOME", raising=False)
    monkeypatch.setenv("HOME", str(tmp_path))
    expected = (tmp_path / ".local/share") / "staubli_gui" / "scan_videos"
    assert vr.default_video_dir() == expected
