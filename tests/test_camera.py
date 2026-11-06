"""Tests de vision/camera_pi.py avec de fausses caméras (pas besoin du matériel)."""
from __future__ import annotations

import http.client
import runpy
import sys
import threading
import time
import types
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional
from unittest import mock

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from vision import camera_pi  # noqa: E402
from vision.camera_pi import FrameStreamer, RobotCamera  # noqa: E402

BLOCKED = {"picamera": None, "picamera.array": None, "picamera2": None, "libcamera": None}


def gradient() -> np.ndarray:
    """Image avec une valeur différente par ligne, pour voir si elle est retournée."""
    frame = np.zeros((128, 160, 3), np.uint8)
    frame[:, :, :] = np.arange(128, dtype=np.uint8)[:, None, None]
    return frame


def wait_until(condition: Any, timeout: float = 3.0) -> None:
    deadline = time.monotonic() + timeout
    while not condition():
        assert time.monotonic() < deadline, "condition jamais atteinte"
        time.sleep(0.01)


# FrameStreamer

@pytest.fixture
def streamer() -> Any:
    instance = FrameStreamer(port=0, fps=0)
    yield instance
    instance.close()


def port_of(instance: FrameStreamer) -> int:
    return instance._server.server_address[1]


def test_publish_n_encode_pas_et_copie_l_image(streamer: FrameStreamer) -> None:
    frame = gradient()
    with mock.patch.object(camera_pi.cv2, "imencode") as imencode:
        streamer.publish(frame)
    imencode.assert_not_called()

    original = frame.copy()
    frame[:] = 0  # le robot peut réutiliser son tableau
    assert np.array_equal(streamer._raw_frame, original)


def test_publish_ne_reveille_personne_sans_client(streamer: FrameStreamer) -> None:
    with mock.patch.object(streamer._cond, "notify_all") as notify:
        streamer.publish(gradient())
        notify.assert_not_called()
        streamer._clients = 1
        streamer.publish(gradient())
        notify.assert_called_once()
    streamer._clients = 0


def test_publish_ne_bloque_pas_pendant_l_encodage(streamer: FrameStreamer) -> None:
    """Un encodage lent ne doit pas ralentir la boucle du robot."""
    streamer.publish(gradient())
    entered, release = threading.Event(), threading.Event()
    real_imencode = cv2.imencode

    def slow_imencode(*args: Any) -> Any:
        entered.set()
        release.wait(5)
        return real_imencode(*args)

    with mock.patch.object(camera_pi.cv2, "imencode", slow_imencode):
        worker = threading.Thread(target=streamer._get_jpeg)
        worker.start()
        assert entered.wait(5)
        started = time.monotonic()
        streamer.publish(gradient())
        elapsed = time.monotonic() - started
        release.set()
        worker.join(5)
    assert elapsed < 0.5


def test_snapshot_encode_une_seule_fois_par_image(streamer: FrameStreamer) -> None:
    url = f"http://127.0.0.1:{port_of(streamer)}/snapshot.jpg"
    with pytest.raises(urllib.error.HTTPError) as error:
        urllib.request.urlopen(url)
    assert error.value.code == 503

    streamer.publish(gradient())
    with mock.patch.object(camera_pi.cv2, "imencode", wraps=cv2.imencode) as imencode:
        bodies = [urllib.request.urlopen(url).read() for _ in range(3)]
        assert imencode.call_count == 1
        streamer.publish(gradient())
        urllib.request.urlopen(url).read()
        assert imencode.call_count == 2
    assert bodies[0] == bodies[1] == bodies[2]
    assert cv2.imdecode(np.frombuffer(bodies[0], np.uint8), cv2.IMREAD_COLOR).shape == (128, 160, 3)


def test_echec_d_encodage_ne_fait_pas_tourner_en_boucle(streamer: FrameStreamer) -> None:
    streamer.publish(gradient())
    with mock.patch.object(camera_pi.cv2, "imencode", return_value=(False, None)) as imencode:
        assert streamer._get_jpeg() == (None, 1)
        assert streamer._get_jpeg() == (None, 1)
    imencode.assert_called_once()


def read_part(response: http.client.HTTPResponse) -> bytes:
    length = 0
    while True:
        line = response.readline()
        if line.lower().startswith(b"content-length:"):
            length = int(line.split(b":")[1])
        if line == b"\r\n" and length:
            return response.read(length)


def test_flux_compte_ses_clients_et_partage_le_cache(streamer: FrameStreamer) -> None:
    connections = [http.client.HTTPConnection("127.0.0.1", port_of(streamer)) for _ in range(2)]
    responses = []
    for connection in connections:
        connection.request("GET", "/stream.mjpg")
        responses.append(connection.getresponse())
    wait_until(lambda: streamer._clients == 2)

    with mock.patch.object(camera_pi.cv2, "imencode", wraps=cv2.imencode) as imencode:
        streamer.publish(gradient())
        first, second = (read_part(response) for response in responses)
        assert imencode.call_count == 1
    assert first == second

    for response in responses:
        response.close()
    # On ne voit le départ d'un client qu'à l'écriture suivante, donc on continue de publier
    wait_until(lambda: streamer._clients == 0 or streamer.publish(gradient()))


def test_flux_sans_image_n_utilise_pas_le_processeur(streamer: FrameStreamer) -> None:
    connection = http.client.HTTPConnection("127.0.0.1", port_of(streamer))
    connection.request("GET", "/stream.mjpg")
    connection.getresponse()
    wait_until(lambda: streamer._clients == 1)
    with mock.patch.object(streamer, "_get_jpeg", wraps=streamer._get_jpeg) as get_jpeg:
        time.sleep(0.2)
    assert get_jpeg.call_count == 0
    connection.close()


def test_fermeture_avec_client_connecte() -> None:
    instance = FrameStreamer(port=0, fps=0)
    connection = http.client.HTTPConnection("127.0.0.1", port_of(instance))
    connection.request("GET", "/stream.mjpg")
    connection.getresponse()
    wait_until(lambda: instance._clients == 1)

    started = time.monotonic()
    instance.close()
    assert time.monotonic() - started < 2.0
    wait_until(lambda: instance._clients == 0)
    instance.publish(gradient())  # sans effet après fermeture
    connection.close()


# Sources d'images

def fake_picamera(calls: Dict[str, Any]) -> Dict[str, types.ModuleType]:
    class PiCamera:
        def __init__(self, **kwargs: Any) -> None:
            calls["init"] = kwargs
            self.closed = False

        def capture_continuous(self, raw: Any, format: str, use_video_port: bool) -> Any:
            def frames() -> Any:
                try:
                    while True:
                        yield types.SimpleNamespace(array=gradient())
                finally:
                    calls["stream_closed"] = True
            return frames()

        def close(self) -> None:
            calls["closed"] = True

        def __setattr__(self, name: str, value: Any) -> None:
            calls.setdefault("attrs", {})[name] = value
            object.__setattr__(self, name, value)

    picamera, array = types.ModuleType("picamera"), types.ModuleType("picamera.array")
    picamera.PiCamera = PiCamera  # type: ignore[attr-defined]
    array.PiRGBArray = lambda cam, size: mock.MagicMock()  # type: ignore[attr-defined]
    return {"picamera": picamera, "picamera.array": array}


def fake_picamera2(calls: Dict[str, Any]) -> Dict[str, Any]:
    class Picamera2:
        sensor_modes: List[Dict[str, Any]] = [
            {"size": (640, 480), "crop_limits": (0, 0, 3280, 2464)},
            {"size": (1640, 1232), "crop_limits": (0, 0, 3280, 2464)},
        ]
        camera_properties = {"ScalerCropMaximum": (0, 0, 3280, 2464)}

        def create_video_configuration(self, **kwargs: Any) -> Dict[str, Any]:
            calls["config"] = kwargs
            return kwargs

        def configure(self, config: Any) -> None:
            pass

        def start(self) -> None:
            calls["started"] = True

        def set_controls(self, controls: Any) -> None:
            pass

        def capture_array(self) -> np.ndarray:
            return gradient()

        def stop(self) -> None:
            calls["stopped"] = True

    picamera2 = types.ModuleType("picamera2")
    picamera2.Picamera2 = Picamera2  # type: ignore[attr-defined]
    libcamera = types.ModuleType("libcamera")
    libcamera.Transform = lambda vflip=0: {"vflip": vflip}  # type: ignore[attr-defined]
    return {"picamera": None, "picamera.array": None, "picamera2": picamera2, "libcamera": libcamera}


def fake_opencv_capture() -> Any:
    capture = mock.MagicMock()
    capture.read.return_value = (True, gradient())
    return capture


def is_upright(frame: Optional[np.ndarray]) -> bool:
    assert frame is not None
    return int(frame[0, 0, 0]) < int(frame[-1, 0, 0])


@pytest.mark.parametrize("vflip", [True, False])
def test_picamera_flip_materiel(vflip: bool, monkeypatch: pytest.MonkeyPatch) -> None:
    calls: Dict[str, Any] = {}
    monkeypatch.setattr(sys, "modules", {**sys.modules, **fake_picamera(calls)})
    camera = RobotCamera(stream=False, vflip=vflip)
    assert camera.kind == "picamera"
    assert calls["init"]["framerate"] == 40
    assert calls["attrs"]["vflip"] is vflip
    assert is_upright(camera.read()), "le flip est matériel : Python ne doit pas retourner l'image"
    camera.close()
    assert calls["closed"] and calls["stream_closed"]


@pytest.mark.parametrize("vflip,expected", [(True, 1), (False, 0)])
def test_picamera2_configuration(vflip: bool, expected: int, monkeypatch: pytest.MonkeyPatch) -> None:
    calls: Dict[str, Any] = {}
    monkeypatch.setattr(sys, "modules", {**sys.modules, **fake_picamera2(calls)})
    camera = RobotCamera(stream=False, vflip=vflip)
    assert camera.kind == "picamera2"
    config = calls["config"]
    assert config["transform"] == {"vflip": expected}
    assert config["buffer_count"] == 1
    assert config["controls"] == {"FrameRate": 40, "FrameDurationLimits": (25000, 25000)}
    assert is_upright(camera.read())
    camera.close()
    assert calls["stopped"]


@pytest.mark.parametrize("vflip", [True, False])
def test_opencv_flip_logiciel(vflip: bool, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(sys, "modules", {**sys.modules, **BLOCKED})
    capture = fake_opencv_capture()
    monkeypatch.setattr(camera_pi.cv2, "VideoCapture", lambda index: capture)
    camera = RobotCamera(stream=False, vflip=vflip)
    assert camera.kind == "opencv"
    assert is_upright(camera.read()) is not vflip
    camera.close()
    capture.release.assert_called_once()


def test_opencv_lecture_ratee(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(sys, "modules", {**sys.modules, **BLOCKED})
    capture = mock.MagicMock()
    capture.read.return_value = (False, None)
    monkeypatch.setattr(camera_pi.cv2, "VideoCapture", lambda index: capture)
    camera = RobotCamera(stream=False)
    assert camera.read() is None
    camera.close()


def test_miroir_horizontal_conserve(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(sys, "modules", {**sys.modules, **BLOCKED})
    source = np.zeros((128, 160, 3), np.uint8)
    source[:, :10] = 255
    capture = mock.MagicMock()
    capture.read.return_value = (True, source)
    monkeypatch.setattr(camera_pi.cv2, "VideoCapture", lambda index: capture)
    frame = RobotCamera(stream=False, vflip=False).read()
    assert frame is not None and frame[0, 0, 0] == 0 and frame[0, -1, 0] == 255


# Exécution directe

def test_script_executable_directement(monkeypatch: pytest.MonkeyPatch, tmp_path: Path, capsys: Any) -> None:
    capture = mock.MagicMock()
    reads = iter([(True, gradient()), (True, gradient()), KeyboardInterrupt()])
    capture.read.side_effect = reads

    monkeypatch.setattr(sys, "modules", {**sys.modules, **BLOCKED})
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(cv2, "VideoCapture", lambda index: capture)

    runpy.run_path(str(ROOT / "vision" / "camera_pi.py"), run_name="__main__")

    assert "Caméra : opencv" in capsys.readouterr().out
    assert (tmp_path / "test_camera.png").exists()
    capture.release.assert_called_once()
