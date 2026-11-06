"""Constantes du robot (valeurs par défaut).

Celles qui sont dans robot/tuning.py se règlent aussi depuis l'interface web. Il faut donc
les lire avec utils.KP et pas avec from robot.utils import KP, sinon le changement n'est pas pris en compte.
"""

# Vitesses, en fraction de la vitesse max des moteurs (entre 0 et 1)
BASE_SPEED = 0.4  # sur la ligne
FORWARD_SPEED = 0.1  # à l'approche d'un croisement, ou quand la ligne est perdue
TURN_SPEED = 0.8  # rotation sur place
MAX_WHEEL_SPEED = 1.0  # au-dessus le PWM déborde sur l'Arduino et la roue ralentit

# Suivi de ligne : vitesse ajoutée à une roue et enlevée à l'autre quand la ligne est au bord de l'image
KP = 0.15

# Durées en secondes
TURN_90_DURATION = 1.5  # rotation de 90°
TURN_180_DURATION = 3.0  # demi-tour
APPROACH_DURATION = 0.9  # approche du centre d'un croisement, à FORWARD_SPEED
DELIVERY_HALF_TURNS = 2  # le tour complet à destination, c'est 2 demi-tours de TURN_180_DURATION
RAMP_TIME_CONSTANT = 0.15  # constante de temps des rampes
LINE_LOST_TIMEOUT = 0.6  # en suivi ou en recherche, on s'arrête si on ne voit plus la ligne pendant ce temps

# Obstacles, distances en cm depuis le capteur ultrason (cdc)
OBSTACLE_SLOWDOWN_CM = 60  # on commence à ralentir en dessous
OBSTACLE_STOP_CM = 20  # arrêt complet
OBSTACLE_RESUME_CM = 25  # une fois arrêté, on ne repart qu'au-dessus de cette distance
OBSTACLE_FILTER_READINGS = 3  # nombre de mesures gardées, on prend la plus petite
OBSTACLE_CONFIRM_SECONDS = 1.0  # bloqué depuis ce temps devant un obstacle, on enlève l'arête du graphe

# Après un obstacle : retour au dernier noeud à vitesse réduite
DETOUR_SPEED = 0.2  # vitesse sur la ligne pendant le retour
DETOUR_APPROACH_DURATION = 0.9  # approche du centre du noeud, à FORWARD_SPEED, en secondes
ALIGN_SPEED = 0.3  # vitesse des roues quand on se recale sur la ligne après le demi-tour
ALIGN_TOLERANCE = 0.1  # écart accepté au milieu de l'image, en fraction de la demi-largeur
ALIGN_SEARCH_DEGREES = 60  # si la ligne n'est pas visible, on la cherche jusqu'à cet angle de chaque côté

# En dessous de cet écart on considère que la consigne est atteinte
# (une exponentielle ne l'atteint jamais vraiment)
SPEED_EPSILON = 0.01

# Serveurs
TELEMETRY_PORT = 8001  # port de l'interface web
MAX_REQUEST_BYTES = 4096
