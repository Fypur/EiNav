"""
Récupération des images de la caméra du vrai robot (Raspberry Pi).

Remplace, sur le robot réel, ce que fait Webots dans `detection` :
    cameraData = camera.getImage()
    image = np.frombuffer(cameraData, np.uint8).reshape((h, w, 4))
    image_bgr = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

Ici, read() renvoie directement une image BGR (numpy, hauteur x largeur x 3),
le même format que `image_bgr` dans `detection`.

Utilisation :
    from camera_pi import RobotCamera
    cam = RobotCamera()
    image_bgr = cam.read()
    cam.close()

Sources essayées dans l'ordre : picamera (vieux Raspbian), picamera2 (Raspbian récent),
webcam OpenCV (pour tester sur PC).
"""
import cv2

RESOLUTION = (160, 128)  # (largeur, hauteur), même taille que les images de data/images_2025


class RobotCamera:
    def __init__(self, resolution=RESOLUTION, framerate=32):
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

    def read(self):
        """Renvoie une image BGR (hauteur x largeur x 3), ou None si la lecture échoue."""
        if self.kind == "picamera":
            frame = next(self._stream).array
            self._raw.truncate(0)
            return frame
        if self.kind == "picamera2":
            return self._cam.capture_array()
        ok, frame = self._cam.read()
        return cv2.resize(frame, self.resolution) if ok else None

    def close(self):
        """À appeler en fin de programme pour libérer la caméra."""
        if self.kind == "picamera":
            self._stream.close()
            self._cam.close()
        elif self.kind == "picamera2":
            self._cam.stop()
        else:
            self._cam.release()


if __name__ == "__main__":
    # Test rapide sur le robot : python camera_pi.py  -> enregistre test_camera.png
    cam = RobotCamera()
    print("Caméra :", cam.kind)
    img = cam.read()
    if img is None:
        print("Échec : aucune image reçue")
    else:
        print("Image reçue :", img.shape)
        cv2.imwrite("test_camera.png", img)
    cam.close()
