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
    from camera_pi import RobotCamera
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
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

RESOLUTION = (160, 128)  # (largeur, hauteur), même taille que les images de data/images_2025

STREAM_PORT = 8000       # doit correspondre à CAMERA_STREAM_PORT dans frontend/app.py
STREAM_FPS = 15          # images/s max envoyées au navigateur (économise le CPU du Pi)
JPEG_QUALITY = 80
BOUNDARY = b"frame"

_TEST_PAGE = b"""<!doctype html><html><head><meta charset="utf-8"><title>Camera robot</title>
<style>body{background:#101b2d;color:#cbd5e1;font-family:sans-serif;text-align:center}
img{width:min(95vw,640px);image-rendering:pixelated;border-radius:12px}</style></head>
<body><h3>Flux camera du robot</h3><img src="/stream.mjpg" alt="flux indisponible"></body></html>"""


class _HTTPServer(ThreadingHTTPServer):
    # Sous Windows, SO_REUSEADDR autorise deux serveurs sur le même port : on le désactive
    # pour que l'erreur « port déjà utilisé » soit bien levée. Sous Linux (Pi), on le garde
    # pour pouvoir relancer le programme immédiatement.
    allow_reuse_address = os.name != "nt"
    daemon_threads = True


class FrameStreamer:
    """Serveur HTTP qui diffuse les dernières images publiées au format MJPEG."""

    def __init__(self, port=STREAM_PORT, fps=STREAM_FPS, quality=JPEG_QUALITY):
        self.port = port
        self._period = 1.0 / fps if fps > 0 else 0.0
        self._quality = int(quality)
        self._cond = threading.Condition()
        self._jpeg = None
        self._frame_id = 0
        self._last_publish = 0.0
        self._closed = False

        streamer = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
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

            def log_message(self, *args):
                pass  # pas de log à chaque requête

        self._server = _HTTPServer(("0.0.0.0", port), Handler)  # OSError si port pris
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

    def publish(self, frame):
        """Encode l'image BGR en JPEG et la rend disponible aux navigateurs connectés."""
        if frame is None or self._closed:
            return
        now = time.monotonic()
        if now - self._last_publish < self._period:
            return  # limitation du débit : on saute cette image
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self._quality])
        if not ok:
            return
        self._last_publish = now
        with self._cond:
            self._jpeg = buf.tobytes()
            self._frame_id += 1
            self._cond.notify_all()

    def _serve_snapshot(self, handler):
        with self._cond:
            jpeg = self._jpeg
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

    def _serve_stream(self, handler):
        handler.send_response(200)
        handler.send_header("Content-Type", "multipart/x-mixed-replace; boundary=" + BOUNDARY.decode())
        handler.send_header("Cache-Control", "no-store")
        handler.send_header("Pragma", "no-cache")
        handler.send_header("Access-Control-Allow-Origin", "*")
        handler.end_headers()
        last_id = -1
        try:
            while True:
                with self._cond:
                    self._cond.wait_for(lambda: self._closed or self._frame_id != last_id, timeout=2.0)
                    if self._closed:
                        return
                    if self._frame_id == last_id or self._jpeg is None:
                        continue  # pas de nouvelle image : le navigateur garde la précédente
                    last_id, jpeg = self._frame_id, self._jpeg
                handler.wfile.write(b"--" + BOUNDARY + b"\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                    + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                handler.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass  # le navigateur a fermé la page

    def close(self):
        with self._cond:
            self._closed = True
            self._cond.notify_all()
        self._server.shutdown()
        self._server.server_close()


class RobotCamera:
    def __init__(self, resolution=RESOLUTION, framerate=32, stream=True, stream_port=STREAM_PORT):
        self.resolution = resolution
        try:
            from picamera import PiCamera
            from picamera.array import PiRGBArray
            self._cam = PiCamera(sensor_mode=2)
            self._cam.resolution = resolution
            self._cam.framerate = framerate
            self._raw = PiRGBArray(self._cam, size=resolution)
            self._stream = self._cam.capture_continuous(self._raw, format="bgr", use_video_port=True)
            self.kind = "picamera"
        except ImportError:
            try:
                from picamera2 import Picamera2
                self._cam = Picamera2()
                self._cam.configure(self._cam.create_video_configuration(
                    main={"size": resolution, "format": "RGB888"}))  # RGB888 = octets BGR pour OpenCV
                self._cam.start()
                self.kind = "picamera2"
            except ImportError:
                self._cam = cv2.VideoCapture(0)
                self.kind = "opencv"

        # Diffusion vers l'interface. Une erreur ici ne doit jamais empêcher le robot de rouler.
        self.streamer = None
        self._publish_raw = True
        if stream:
            try:
                self.streamer = FrameStreamer(port=stream_port)
                print(f"[camera] flux vidéo : http://<IP_du_Pi>:{stream_port}/stream.mjpg")
            except OSError as exc:
                print(f"[camera] flux vidéo désactivé (port {stream_port} indisponible : {exc})")

    def read(self):
        """Renvoie une image BGR (hauteur x largeur x 3), ou None si la lecture échoue."""
        frame = self._read_frame()
        if self._publish_raw and self.streamer is not None:
            self.streamer.publish(frame)
        return frame

    def publish(self, frame):
        """Diffuse une image (par ex. annotée avec les détections) à la place de l'image brute."""
        self._publish_raw = False
        if self.streamer is not None:
            self.streamer.publish(frame)

    def _read_frame(self):
        if self.kind == "picamera":
            frame = cv2.flip(next(self._stream).array,0)
            self._raw.truncate(0)
            return frame
        if self.kind == "picamera2":
            return self._cam.capture_array()
        ok, frame = self._cam.read()
        return cv2.resize(frame, self.resolution) if ok else None

    def close(self):
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
    # Test rapide sur le robot : python camera_pi.py
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
