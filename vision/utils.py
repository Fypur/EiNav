"""Constantes de la caméra et du traitement d'image (valeurs par défaut).

Celles qui sont dans robot/tuning.py se règlent aussi depuis l'interface web.
"""

# Caméra
RESOLUTION = (160, 128)  # (largeur, hauteur), même taille que les images de data/images_2025
MAX_FULL_FOV_FPS = 40  # cadence max des modes capteur plein champ (binning 2x2)

# Flux vidéo
STREAM_PORT = 8000  # doit correspondre à STREAM_PORT dans frontend/live.html
STREAM_FPS = 15  # images/s max vers le navigateur, pour ne pas surcharger le Pi
JPEG_QUALITY = 80
BOUNDARY = b"frame"

# Détection de la ligne
# Zone d'intérêt (le rectangle bleu), en fraction de la hauteur et de la largeur de l'image
ROI_TOP_RATIO = 0.6
ROI_BOTTOM_RATIO = 1.0
ROI_LEFT_RATIO = 0.0
ROI_RIGHT_RATIO = 1.0
ROI_MIN_PIXELS = 10  # taille min de la zone, si les réglages se croisent (bas au-dessus du haut par exemple)
BINARY_THRESHOLD = 140  # niveau de gris (0-255) à partir duquel un pixel fait partie de la ligne
MIN_LINE_AREA = 50  # surface min (pixels) pour accepter une ligne
# Largeur min de la bande blanche d'un croisement, en fraction de la largeur de l'image. Sur 227 images réelles,
# une ligne en biais prise pour un croisement ne dépasse pas 0,22 et un croisement centré commence à 0,54.
# Un T en bord de grille avec la ligne décalée descend vers 0,37, d'où un seuil assez bas.
INTERSECTION_MIN_WIDTH_RATIO = 0.3
CROSSING_REQUIRE_TOP = True  # méthode forte : la ligne doit aussi toucher le haut de la zone (False : version de Farès)
CROSSING_USE_HOUGH = True  # deuxième méthode, si la méthode forte ne voit rien
# Deuxième méthode (Hough) : croisement si on voit une horizontale et une verticale dans la zone d'intérêt.
# Calé sur les 227 images réelles, aussi tournées jusqu'à 25°, décalées et plus ou moins lumineuses.
HOUGH_HORIZONTAL_MAX_ANGLE = 20  # degrés, au-dessus ce n'est plus une ligne horizontale
HOUGH_HORIZONTAL_MIN_WIDTH_RATIO = 0.2  # longueur min, en fraction de la largeur de l'image (les marches du masque font moins)
HOUGH_VERTICAL_MIN_ANGLE = 65  # degrés, en dessous ce n'est plus une ligne verticale
HOUGH_VERTICAL_MIN_HEIGHT_RATIO = 0.4  # longueur min, en fraction de la hauteur de la zone
HOUGH_VOTES = 12  # nombre de pixels alignés pour que Hough garde un segment
HOUGH_MAX_GAP = 3  # trou max (pixels) dans un segment
ALIGN_ROWS = 5  # rangées lointaines utilisées pour se recaler sur la ligne après un demi-tour
ALIGN_MAX_WIDTH_JUMP = 4  # pixels, au-delà d'une rangée à l'autre ce n'est plus la ligne mais une branche
BLUR_KERNEL = (9, 9)  # dimensions impaires
MORPH_KERNEL = (9, 9)

# Couleurs de l'image traitée (BGR)
ROI_COLOR = (255, 200, 0)
LINE_COLOR = (0, 200, 0)
TARGET_COLOR = (0, 0, 255)
INTERSECTION_COLOR = (0, 165, 255)
