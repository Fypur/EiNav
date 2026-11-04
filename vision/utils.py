"""Constantes de la caméra et du traitement d'image. Les valeurs ci-dessous sont celles par défaut.

Celles listées dans robot/tuning.py se règlent aussi depuis l'interface web.
"""

# Caméra.
RESOLUTION = (160, 128)  # (largeur, hauteur), même taille que les images de data/images_2025
MAX_FULL_FOV_FPS = 40  # cadence max des modes capteur plein champ (binning 2x2)

# Flux vidéo.
STREAM_PORT = 8000  # doit correspondre à CAMERA_STREAM_PORT dans frontend/utils.py
STREAM_FPS = 15  # images/s max envoyées au navigateur (économise le CPU du Pi)
JPEG_QUALITY = 80
BOUNDARY = b"frame"

# Détection de la ligne.
ROI_TOP_RATIO = 0.6  # la zone d'intérêt commence à cette fraction de la hauteur de l'image
BINARY_THRESHOLD = 140  # niveau de gris (0-255) au-dessus duquel un pixel fait partie de la ligne
MIN_LINE_AREA = 50  # surface minimale (pixels) pour accepter une ligne
BLUR_KERNEL = (9, 9)  # dimensions impaires
MORPH_KERNEL = (9, 9)

# Couleurs de l'image traitée (BGR).
ROI_COLOR = (255, 200, 0)
LINE_COLOR = (0, 200, 0)
TARGET_COLOR = (0, 0, 255)
INTERSECTION_COLOR = (0, 165, 255)
