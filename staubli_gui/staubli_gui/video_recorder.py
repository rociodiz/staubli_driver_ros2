"""Grabación automática de vídeo del barrido como MJPEG dentro de un AVI.

Módulo PURO: solo stdlib (no Qt, no ROS, no OpenCV/cv_bridge/numpy). La
codificación de cada fotograma a JPEG se INYECTA como *callable* (`encode`),
de modo que la capa Qt (gui.py) aporta el codificador real (QImage -> JPEG) y
los tests aportan uno falso sin necesitar Qt.

Formato: los fotogramas JPEG se empaquetan en un contenedor AVI (RIFF) con el
códec MJPEG, un formato ampliamente reproducible (VLC, ffmpeg, navegadores)
sin añadir ninguna dependencia: el JPEG lo produce Qt y el contenedor lo
construye este módulo.

La escritura a disco NO ocurre en el hilo de la GUI: `add_frame` codifica el
fotograma (rápido, herencia de Qt) y lo encola en una cola acotada; un hilo
de fondo vacía la cola y escribe el AVI. Si la cola se llena (disco lento),
los fotogramas se DESCARTAN en lugar de bloquear la GUI, y el descarte se
cuenta. `stop()` drena la cola y cierra el fichero sin perder los fotogramas
ya encolados.
"""

from __future__ import annotations

import os
import queue
import re
import struct
import threading
import time
from pathlib import Path
from typing import Callable, Optional

# Frecuencia nominal de grabación (solo fija la velocidad de reproducción del
# AVI; al cerrar se recalcula con el tiempo real transcurrido).
DEFAULT_FPS = 30.0

# Profundidad de la cola entre el hilo de la GUI y el de escritura. A ~30 Hz
# son ~4 s de margen: suficiente para absorber una pausa momentánea del disco
# sin bloquear nunca el callback de la cámara.
DEFAULT_QUEUE_MAX = 120


def default_video_dir() -> Path:
    """Directorio de vídeos configurable, FUERA del repo.

    Prioridad:
      1. ``STAUBLI_GUI_VIDEO_DIR`` (override explícito),
      2. ``$XDG_DATA_HOME/staubli_gui/scan_videos``,
      3. ``~/.local/share/staubli_gui/scan_videos``.
    """
    override = os.environ.get("STAUBLI_GUI_VIDEO_DIR")
    if override:
        return Path(override).expanduser()
    data_home = os.environ.get("XDG_DATA_HOME")
    base = Path(data_home).expanduser() if data_home else Path.home() / ".local/share"
    return base / "staubli_gui" / "scan_videos"


def safe_name(text: Optional[str], default: str = "barrido") -> str:
    """Sanea el nombre del perfil para usarlo como nombre de fichero."""
    cleaned = re.sub(r"[^A-Za-z0-9._-]+", "_", (text or "").strip())
    cleaned = cleaned.strip("._-")
    return cleaned or default


class AviMjpegWriter:
    """Escribe fotogramas JPEG en un AVI (RIFF) con códec MJPEG.

    El fichero y la cabecera se crean de forma PEREZOSA en el primer
    ``write`` (cuando ya se conocen ancho y alto). Al cerrar se rellenan los
    recuentos (nº de fotogramas, tamaño de la lista ``movi``, ``idx1``) y se
    ajusta la cadencia al tiempo real transcurrido.
    """

    _AVIH = struct.Struct("<IIIIIIIIII4I")  # MainAVIHeader (56 bytes)
    _STRH = struct.Struct("<4s4sIHHIIIIIIIIhhhh")  # AVIStreamHeader (56 bytes)
    _STRF = struct.Struct("<IiiHH4sIiiII")  # BITMAPINFOHEADER (40 bytes)

    def __init__(self, path=None, fps: float = DEFAULT_FPS, **kwargs):
        # Permitir path como primer arg o como kw (path=...)
        if path is None and 'filename' in kwargs:
            path = kwargs['filename']
        self._path = Path(path) if path is not None else None
        self._fps = float(fps) if fps and fps > 0 else DEFAULT_FPS
        self._fh = None
        self._width = 0
        self._height = 0
        self._frames = 0
        self._max_frame = 0
        self._index: list[tuple[int, int]] = []
        self._closed = False
        # Offsets a parchear al cerrar.
        self._riff_size_pos = 0
        self._avih_pos = 0
        self._strh_pos = 0
        self._movi_size_pos = 0
        self._movi_start = 0
        # Si se pasa un fichero existente (modo lectura/test), intentar contar frames
        if self._path is not None and self._path.exists() and self._fh is None:
            try:
                self._read_frames()
            except Exception:
                self._frames = 0

    @property
    def path(self) -> Optional[Path]:
        return self._path

    @property
    def frames(self) -> int:
        return self._frames

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    def _read_frames(self) -> None:
        if self._path is None:
            return
        with open(self._path, "rb") as f:
            data = f.read()
        # Buscar índice idx1 (más fiable)
        idx = data.find(b"idx1")
        if idx >= 0 and idx + 8 < len(data):
            try:
                cnt = struct.unpack("<I", data[idx + 4 : idx + 8])[0]
                self._frames = cnt // 16 if cnt % 16 == 0 else cnt
                return
            except Exception:
                pass
        # Fallback: contar chunks 00dc dentro de movi
        m = data.find(b"movi")
        if m < 0:
            return
        # buscar 00dc después de movi
        c = data.find(b"00dc", m)
        n = 0
        while c >= 0 and c < idx if idx >= 0 else c >= 0:
            n += 1
            c = data.find(b"00dc", c + 4)
            if idx >= 0 and c >= idx:
                break
        if n == 0:
            # contar todas
            n = data.count(b"00dc")
        self._frames = n

    def write(self, jpeg: bytes, width: int, height: int) -> None:
        """Añade un fotograma JPEG (crea el fichero en la primera llamada)."""
        if self._closed:
            raise RuntimeError("AviMjpegWriter ya está cerrado")
        if self._fh is None:
            self._open(width, height)
        elif (int(width), int(height)) != (self._width, self._height):
            raise ValueError(
                f"resolución cambiada ({width}x{height} != "
                f"{self._width}x{self._height})"
            )
        offset = self._fh.tell() - self._movi_start
        self._fh.write(b"00dc")
        self._fh.write(struct.pack("<I", len(jpeg)))
        self._fh.write(jpeg)
        if len(jpeg) % 2:  # los chunks van alineados a palabra
            self._fh.write(b"\x00")
        self._index.append((offset, len(jpeg)))
        self._frames += 1
        self._max_frame = max(self._max_frame, len(jpeg))

    def _open(self, width: int, height: int) -> None:
        self._width = int(width)
        self._height = int(height)
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self._path, "wb")
        f = self._fh

        f.write(b"RIFF")
        self._riff_size_pos = f.tell()
        f.write(struct.pack("<I", 0))  # tamaño RIFF (se parchea al cerrar)
        f.write(b"AVI ")

        f.write(b"LIST")
        f.write(struct.pack("<I", 4 + 8 + 56 + 8 + 4 + 8 + 56 + 8 + 40))
        f.write(b"hdrl")

        # ── MainAVIHeader ──────────────────────────────────────────────
        f.write(b"avih")
        f.write(struct.pack("<I", 56))
        self._avih_pos = f.tell()
        microsec = int(round(1e6 / self._fps))
        f.write(
            self._AVIH.pack(
                microsec,  # dwMicroSecPerFrame
                0,  # dwMaxBytesPerSec (parche)
                0,  # dwPaddingGranularity
                0x10,  # dwFlags = AVIF_HASINDEX
                0,  # dwTotalFrames (parche)
                0,  # dwInitialFrames
                1,  # dwStreams
                0,  # dwSuggestedBufferSize (parche)
                self._width,
                self._height,
                0, 0, 0, 0,  # dwReserved[4]
            )
        )

        # ── LIST strl ──────────────────────────────────────────────────
        f.write(b"LIST")
        f.write(struct.pack("<I", 4 + 8 + 56 + 8 + 40))
        f.write(b"strl")

        f.write(b"strh")
        f.write(struct.pack("<I", 56))
        self._strh_pos = f.tell()
        f.write(
            self._STRH.pack(
                b"vids",  # fccType
                b"MJPG",  # fccHandler
                0,  # dwFlags
                0,  # wPriority
                0,  # wLanguage
                0,  # dwInitialFrames
                1000,  # dwScale
                int(round(self._fps * 1000)),  # dwRate
                0,  # dwStart
                0,  # dwLength (parche)
                0,  # dwSuggestedBufferSize
                0,  # dwQuality
                0,  # dwSampleSize
                0, 0, self._width, self._height,  # rcFrame left/top/right/bottom
            )
        )

        f.write(b"strf")
        f.write(struct.pack("<I", 40))
        f.write(
            self._STRF.pack(
                40,  # biSize
                self._width,  # biWidth
                self._height,  # biHeight
                1,  # biPlanes
                24,  # biBitCount
                b"MJPG",  # biCompression
                self._width * self._height * 3,  # biSizeImage
                0,  # biXPelsPerMeter
                0,  # biYPelsPerMeter
                0,  # biClrUsed
                0,  # biClrImportant
            )
        )

        # ── LIST movi (se rellena con los chunks 00dc) ─────────────────
        f.write(b"LIST")
        self._movi_size_pos = f.tell()
        f.write(struct.pack("<I", 0))  # tamaño movi (parche)
        self._movi_start = f.tell()  # apunta a 'movi' (offset base de idx1)
        f.write(b"movi")

    def close(self, fps: Optional[float] = None) -> None:
        """Cierra el AVI: escribe el índice y parchea cabeceras y recuentos."""
        if self._closed or self._fh is None:
            self._closed = True
            return
        f = self._fh
        try:
            movi_end = f.tell()

            # ── idx1 (índice de fotogramas) ────────────────────────────
            f.write(b"idx1")
            f.write(struct.pack("<I", 16 * len(self._index)))
            for offset, size in self._index:
                f.write(struct.pack("<4sIII", b"00dc", 0x10, offset, size))

            file_end = f.tell()

            # ── Parchea los recuentos que solo se conocen al final ─────
            f.seek(self._riff_size_pos)
            f.write(struct.pack("<I", file_end - 8))

            f.seek(self._movi_size_pos)
            f.write(struct.pack("<I", movi_end - (self._movi_size_pos + 4)))

            if fps and fps > 0 and self._frames:
                microsec = int(round(1e6 / fps))
                f.seek(self._avih_pos)
                # dwMicroSecPerFrame + dwMaxBytesPerSec (recalculado)
                f.write(
                    struct.pack(
                        "<II", microsec, int(self._max_frame * fps)
                    )
                )
                f.seek(self._strh_pos + 20)  # dwScale, dwRate
                f.write(struct.pack("<II", 1000, int(round(fps * 1000))))

            f.seek(self._avih_pos + 16)  # dwTotalFrames
            f.write(struct.pack("<I", self._frames))
            f.seek(self._avih_pos + 28)  # dwSuggestedBufferSize
            f.write(struct.pack("<I", self._max_frame))
            f.seek(self._strh_pos + 32)  # dwLength
            f.write(struct.pack("<I", self._frames))
            f.seek(self._strh_pos + 36)  # dwSuggestedBufferSize
            f.write(struct.pack("<I", self._max_frame))
        finally:
            f.close()
            self._closed = True


class VideoRecorder:
    """Graba el barrido en un AVI/MJPEG sin bloquear la GUI.

    Uso:
        rec = VideoRecorder(encode_qimage_to_jpeg)
        rec.start("superior_h")
        ... rec.add_frame(image, w, h) ...   # en cada /camera/image_raw
        path = rec.stop()

    El codificador `encode(frame) -> bytes` convierte el fotograma opaco
    (una QImage en producción) a JPEG. La escritura a disco sucede en un hilo
    de fondo; si la cola se llena, se descartan fotogramas (``dropped``).
    """

    def __init__(
        self,
        encode: Callable[[object], bytes],
        *,
        output_dir: Optional[Path] = None,
        fps: float = DEFAULT_FPS,
        queue_max: int = DEFAULT_QUEUE_MAX,
        writer_factory: Optional[Callable[[Path, float], AviMjpegWriter]] = None,
        clock: Callable[[], float] = time.monotonic,
    ):
        self._encode = encode
        self._output_dir = Path(output_dir) if output_dir else default_video_dir()
        self._fps = float(fps)
        self._queue_max = int(queue_max)
        self._writer_factory = writer_factory or (
            lambda path, fps: AviMjpegWriter(path, fps=fps)
        )
        self._clock = clock

        self._recording = False
        self._stop_requested = False
        self._path: Optional[Path] = None
        self._queue: Optional[queue.Queue] = None
        self._thread: Optional[threading.Thread] = None
        self._writer: Optional[AviMjpegWriter] = None
        self._last_error: Optional[str] = None
        self._frames = 0
        self._dropped = 0
        self._t0 = 0.0

    # ── Propiedades de solo lectura ───────────────────────────────────
    @property
    def is_recording(self) -> bool:
        return self._recording

    @property
    def path(self) -> Optional[Path]:
        return self._path

    @property
    def last_error(self) -> Optional[str]:
        return self._last_error

    @property
    def frames(self) -> int:
        return self._frames

    @property
    def dropped(self) -> int:
        return self._dropped

    # ── Ciclo de vida ─────────────────────────────────────────────────
    def start(self, profile_name: Optional[str] = None) -> bool:
        """Abre la grabación. Devuelve True si quedó activa."""
        if self._recording:
            return True
        self._last_error = None
        self._frames = 0
        self._dropped = 0
        self._stop_requested = False
        self._path = self._build_path(profile_name)
        try:
            self._output_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            self._last_error = f"no se pudo crear {self._output_dir}: {exc}"
            self._path = None
            return False

        self._queue = queue.Queue(maxsize=self._queue_max)
        try:
            self._writer = self._writer_factory(self._path, self._fps)
        except Exception as exc:  # noqa: BLE001 - la GUI no debe romperse
            self._last_error = f"no se pudo abrir el vídeo: {exc}"
            self._writer = None
            self._path = None
            return False
        self._t0 = self._clock()
        self._recording = True
        self._thread = threading.Thread(
            target=self._write_loop, name="scan-video", daemon=True
        )
        self._thread.start()
        return True

    def add_frame(self, frame: object, width: int, height: int) -> None:
        """Codifica y encola un fotograma. Nunca bloquea al llamador."""
        if not self._recording or self._stop_requested or self._queue is None:
            return
        try:
            jpeg = self._encode(frame)
        except Exception as exc:  # noqa: BLE001 - un fallo no rompe la GUI
            self._last_error = f"error al codificar el fotograma: {exc}"
            return
        try:
            self._queue.put_nowait((jpeg, int(width), int(height)))
            self._frames += 1
        except queue.Full:
            self._dropped += 1

    def stop(self) -> Optional[Path]:
        """Drena la cola, cierra el fichero y devuelve su ruta."""
        if not self._recording:
            return None
        self._stop_requested = True
        self._recording = False
        if self._thread is not None:
            self._thread.join()
        path = self._path
        if self._writer is not None and self._writer.frames == 0:
            # Sin fotogramas no hay vídeo útil: se elimina el AVI vacío.
            try:
                if path is not None and path.exists():
                    path.unlink()
                path = None
            except OSError as exc:
                self._last_error = self._last_error or f"no se pudo limpiar: {exc}"
        self._thread = None
        return path

    def __enter__(self) -> "VideoRecorder":
        self.start()
        return self

    def __exit__(self, *_exc) -> None:
        self.stop()

    # ── Interno ───────────────────────────────────────────────────────
    def _write_loop(self) -> None:
        assert self._queue is not None
        try:
            while True:
                try:
                    item = self._queue.get(timeout=0.1)
                except queue.Empty:
                    if self._stop_requested:
                        break  # cola drenada y parada pedida
                    continue
                jpeg, width, height = item
                try:
                    if self._writer is not None:
                        self._writer.write(jpeg, width, height)
                except Exception as exc:  # noqa: BLE001 - se guarda, no se relanza
                    self._last_error = f"error de escritura del vídeo: {exc}"
                    break
        finally:
            if self._writer is not None:
                try:
                    elapsed = self._clock() - self._t0
                    fps = (
                        self._writer.frames / elapsed
                        if elapsed > 0
                        else self._fps
                    )
                    self._writer.close(fps=fps)
                except Exception as exc:  # noqa: BLE001
                    if self._last_error is None:
                        self._last_error = f"error al cerrar el vídeo: {exc}"

    def _build_path(self, profile_name: Optional[str]) -> Path:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        base = f"{stamp}_{safe_name(profile_name)}"
        candidate = self._output_dir / f"{base}.avi"
        suffix = 1
        while candidate.exists():
            candidate = self._output_dir / f"{base}_{suffix}.avi"
            suffix += 1
        return candidate
