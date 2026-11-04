"""
Récupération des images de la caméra du vrai robot (Raspberry Pi)
et diffusion du flux vidéo vers l'interface utilisateur (frontend/app.py).

Remplace, sur le robot réel, ce que fait Webots dans `detection` :
    cameraData = camera.getImage()
    image = np.frombuffer(cameraData, np.uint8).reshape((h, w, 4))
    image_bgr = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

Ici, read() renvoie directement une image BGR (numpy, hauteur x largeur x 3),
le même format que `image_bgr` dans `detection`.

Utilisation :
    from vision.camera_pi import RobotCamera
    cam = RobotCamera()
    image_bgr = cam.read()          # l'image est aussi envoyée au flux vidéo
    cam.publish(image_annotee)      # facultatif : diffuser plutôt l'image avec les détections
    cam.close()

Flux vidéo (défi 4.1) :
    La caméra ne peut être ouverte que par un seul programme : c'est donc le programme du
    robot qui diffuse les images, via un petit serveur HTTP MJPEG lancé dans un thread.
      http://<IP_du_Pi>:8000/stream.mjpg   flux continu (balise <img> dans l'interface)
      http://<IP_du_Pi>:8000/snapshot.jpg  dernière image
      http://<IP_du_Pi>:8000/              page de test
    Par défaut, chaque image lue par read() est diffusée. Dès que publish() est appelé,
    le flux montre uniquement les images passées à publish() (ex. avec ligne et croisements
    dessinés par-dessus).

Sources essayées dans l'ordre : picamera (vieux Raspbian), picamera2 (Raspbian récent),
webcam OpenCV (pour tester sur PC).
"""
from __future__ import annotations

import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Optional, Tuple

import cv2
import numpy as np

RESOLUTION = (160, 128)  # (largeur, hauteur), même taille que les images de data/images_2025
MAX_FULL_FOV_FPS = 40    # cadence max des modes capteur plein champ (binning 2x2)

STREAM_PORT = 8000       # doit correspondre à CAMERA_STREAM_PORT dans frontend/app.py
STREAM_FPS = 15          # images/s max envoyées au navigateur (économise le CPU du Pi)
JPEG_QUALITY = 80
BOUNDARY = b"frame"

_TEST_PAGE = b"""<!doctype html><html><head><meta charset="utf-8"><title>Camera robot</title>
<style>html,body{margin:0;height:100%;background:#000;color:#cbd5e1;font-family:sans-serif}
img{display:block;width:100vw;height:100vh;object-fit:contain;image-rendering:pixelated}</style></head>
<body><img src="/stream.mjpg" alt="flux camera indisponible"></body></html>"""


class _HTTPServer(ThreadingHTTPServer):
    # Sous Windows, SO_REUSEADDR autorise deux serveurs sur le même port : on le désactive
    # pour que l'erreur « port déjà utilisé » soit bien levée. Sous Linux (Pi), on le garde
    # pour pouvoir relancer le programme immédiatement.
    allow_reuse_address = os.name != "nt"
    daemon_threads = True


class FrameStreamer:
    """Serveur HTTP qui diffuse les dernières images publiées au format MJPEG."""

    def __init__(self, port: int = STREAM_PORT, fps: int = STREAM_FPS, quality: int = JPEG_QUALITY) -> None:
        self.port = port
        self._period = 1.0 / fps if fps > 0 else 0.0
        self._quality = int(quality)
        self._cond = threading.Condition()
        self._raw_frame: Optional[np.ndarray] = None
        self._frame_id = 0
        self._clients = 0
        self._last_publish = 0.0
        self._closed = False
        # Protège le cache JPEG. Distinct de _cond : encoder en tenant _cond bloquerait
        # publish(), donc la boucle du robot.
        self._encode_lock = threading.Lock()
        self._jpeg: Optional[bytes] = None
        self._encoded_id = 0

        streamer = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                path = self.path.split("?", 1)[0]
                if path == "/stream.mjpg":
                    streamer._serve_stream(self)
                elif path == "/snapshot.jpg":
                    streamer._serve_snapshot(self)
                elif path in ("/", "/index.html"):
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(_TEST_PAGE)))
                    self.end_headers()
                    self.wfile.write(_TEST_PAGE)
                else:
                    self.send_error(404)

            def log_message(self, *args: Any) -> None:
                pass  # pas de log à chaque requête

        self._server = _HTTPServer(("0.0.0.0", port), Handler)  # OSError si port pris
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

    def publish(self, frame: Optional[np.ndarray]) -> None:
        """Rend l'image BGR disponible aux navigateurs. L'encodage JPEG se fait dans les threads HTTP."""
        if frame is None or self._closed:
            return
        now = time.monotonic()
        if now - self._last_publish < self._period:
            return  # limitation du débit : on saute cette image
        self._last_publish = now
        raw_frame = frame.copy()
        with self._cond:
            self._raw_frame = raw_frame
            self._frame_id += 1
            if self._clients > 0:
                self._cond.notify_all()

    def _get_jpeg(self) -> Tuple[Optional[bytes], int]:
        """Dernière image en JPEG et son numéro. Chaque image n'est encodée qu'une seule fois."""
        with self._encode_lock:
            with self._cond:
                raw_frame, frame_id = self._raw_frame, self._frame_id
            if raw_frame is not None and frame_id != self._encoded_id:
                # raw_frame est une copie privée, jamais modifiée : pas besoin de verrou ici.
                ok, buf = cv2.imencode(".jpg", raw_frame, [cv2.IMWRITE_JPEG_QUALITY, self._quality])
                self._encoded_id = frame_id  # même en cas d'échec, pour ne pas réessayer en boucle
                if ok:
                    self._jpeg = buf.tobytes()
            return self._jpeg, frame_id

    def _serve_snapshot(self, handler: BaseHTTPRequestHandler) -> None:
        jpeg, _ = self._get_jpeg()
        if jpeg is None:
            handler.send_error(503, "Aucune image disponible")
            return
        handler.send_response(200)
        handler.send_header("Content-Type", "image/jpeg")
        handler.send_header("Content-Length", str(len(jpeg)))
        handler.send_header("Cache-Control", "no-store")
        handler.send_header("Access-Control-Allow-Origin", "*")
        handler.end_headers()
        handler.wfile.write(jpeg)

    def _serve_stream(self, handler: BaseHTTPRequestHandler) -> None:
        handler.send_response(200)
        handler.send_header("Content-Type", "multipart/x-mixed-replace; boundary=" + BOUNDARY.decode())
        handler.send_header("Cache-Control", "no-store")
        handler.send_header("Pragma", "no-cache")
        handler.send_header("Access-Control-Allow-Origin", "*")
        handler.end_headers()
        last_id = 0  # _frame_id vaut 0 tant qu'aucune image n'a été publiée
        with self._cond:
            self._clients += 1
        try:
            while True:
                with self._cond:
                    self._cond.wait_for(lambda: self._closed or self._frame_id != last_id, timeout=2.0)
                    if self._closed:
                        return
                    if self._frame_id == last_id:
                        continue  # pas de nouvelle image : le navigateur garde la précédente
                jpeg, last_id = self._get_jpeg()
                if jpeg is None:
                    continue
                handler.wfile.write(b"--" + BOUNDARY + b"\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                    + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                handler.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass  # le navigateur a fermé la page
        finally:
            with self._cond:
                self._clients -= 1

    def close(self) -> None:
        with self._cond:
            self._closed = True
            self._cond.notify_all()
        self._server.shutdown()
        self._server.server_close()


def _full_fov_mode_size(picam2: Any) -> Optional[Tuple[int, int]]:
    """picamera2 : taille du mode capteur le plus rapide parmi ceux qui couvrent tout le capteur
    (None si les modes ne peuvent pas être lus : libcamera choisira alors seul)."""
    try:
        modes = picam2.sensor_modes
        area = lambda m: m["crop_limits"][2] * m["crop_limits"][3]
        widest = max(area(m) for m in modes)
        full_fov = [m for m in modes if area(m) == widest]
        return min(full_fov, key=lambda m: m["size"][0] * m["size"][1])["size"]
    except Exception as exc:  # pilote/caméra exotique : on ne bloque pas le robot
        print(f"[camera] modes capteur illisibles ({exc}), champ de vision non garanti")
        return None


class RobotCamera:
    def __init__(self, resolution: Tuple[int, int] = RESOLUTION, framerate: int = MAX_FULL_FOV_FPS,
                 stream: bool = True, stream_port: int = STREAM_PORT, vflip: bool = True) -> None:
        self.resolution = resolution
        self._vflip = vflip
        target_fps = min(framerate, MAX_FULL_FOV_FPS)
        self._cam: Any
        try:
            from picamera import PiCamera
            from picamera.array import PiRGBArray
            # Champ de vision maximal : sensor_mode=4 = capteur entier lu en binning 2x2
            # (caméra v1 : 1296x972, v2 : 1640x1232), jusqu'à ~40 images/s.
            # Le mode 2 (pleine résolution) est limité à 15 images/s : en demandant plus,
            # le firmware bascule sur un mode rapide RECADRÉ au centre du capteur.
            # Mode, résolution et cadence sont passés ensemble pour que le mode soit respecté.
            self._cam = PiCamera(sensor_mode=4, resolution=resolution, framerate=target_fps)
            self._cam.zoom = (0.0, 0.0, 1.0, 1.0)   # aucun recadrage numérique
            self._cam.vflip = vflip                 # flip par l'ISP, sans coût côté Python
            self._raw = PiRGBArray(self._cam, size=resolution)
            self._stream = self._cam.capture_continuous(self._raw, format="bgr", use_video_port=True)
            self.kind = "picamera"
        except ImportError:
            try:
                import libcamera
                from picamera2 import Picamera2
                self._cam = Picamera2()
                # Sans précision, libcamera choisit le plus petit mode capteur (souvent recadré) :
                # on impose un mode qui couvre tout le capteur, puis un recadrage maximal.
                raw_size = _full_fov_mode_size(self._cam)
                # Durée d'image fixe : l'auto-exposition ne peut pas faire chuter la cadence.
                frame_duration_us = int(1_000_000 / target_fps)
                config = self._cam.create_video_configuration(
                    main={"size": resolution, "format": "RGB888"},  # RGB888 = octets BGR pour OpenCV
                    raw={"size": raw_size} if raw_size else None,
                    transform=libcamera.Transform(vflip=int(vflip)),
                    buffer_count=2,  # moins d'images en file d'attente, donc moins de latence
                    controls={"FrameRate": target_fps,
                              "FrameDurationLimits": (frame_duration_us, frame_duration_us)})
                self._cam.configure(config)
                self._cam.start()
                try:
                    self._cam.set_controls({"ScalerCrop": self._cam.camera_properties["ScalerCropMaximum"]})
                except (KeyError, RuntimeError):
                    pass
                self.kind = "picamera2"
            except ImportError:
                self._cam = cv2.VideoCapture(0)
                self.kind = "opencv"

        # Diffusion vers l'interface. Une erreur ici ne doit jamais empêcher le robot de rouler.
        self.streamer: Optional[FrameStreamer] = None
        self._publish_raw = True
        if stream:
            try:
                self.streamer = FrameStreamer(port=stream_port)
                print(f"[camera] flux vidéo : http://<IP_du_Pi>:{stream_port}/stream.mjpg")
            except OSError as exc:
                print(f"[camera] flux vidéo désactivé (port {stream_port} indisponible : {exc})")

    def read(self) -> Optional[np.ndarray]:
        """Renvoie une image BGR (hauteur x largeur x 3), ou None si la lecture échoue."""
        frame = self._read_frame()
        if frame is not None:
            frame = cv2.flip(frame, 1)  # miroir horizontal
        if self._publish_raw and self.streamer is not None:
            self.streamer.publish(frame)
        return frame

    def publish(self, frame: np.ndarray) -> None:
        """Diffuse une image (par ex. annotée avec les détections) à la place de l'image brute."""
        self._publish_raw = False
        if self.streamer is not None:
            self.streamer.publish(frame)

    def _read_frame(self) -> Optional[np.ndarray]:
        if self.kind == "picamera":
            frame = next(self._stream).array
            self._raw.truncate(0)
            return frame
        if self.kind == "picamera2":
            return self._cam.capture_array()
        ok, frame = self._cam.read()
        if not ok:
            return None
        frame = cv2.resize(frame, self.resolution)
        return cv2.flip(frame, 0) if self._vflip else frame

    def close(self) -> None:
        """À appeler en fin de programme pour libérer la caméra et arrêter le flux."""
        if self.streamer is not None:
            self.streamer.close()
            self.streamer = None
        if self.kind == "picamera":
            self._stream.close()
            self._cam.close()
        elif self.kind == "picamera2":
            self._cam.stop()
        else:
            self._cam.release()


if __name__ == "__main__":
    # Test rapide sur le robot : python vision/camera_pi.py
    #   -> enregistre test_camera.png puis diffuse le flux jusqu'à Ctrl+C
    cam = RobotCamera()
    print("Caméra :", cam.kind)
    img = cam.read()
    if img is None:
        print("Échec : aucune image reçue")
    else:
        print("Image reçue :", img.shape)
        cv2.imwrite("test_camera.png", img)
        print(f"Ouvrir http://<IP_du_Pi>:{STREAM_PORT}/ dans un navigateur (Ctrl+C pour arrêter)")
        try:
            while True:
                cam.read()
        except KeyboardInterrupt:
            pass
    cam.close()
