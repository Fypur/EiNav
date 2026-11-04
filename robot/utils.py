"""Constantes de commande du robot. Les valeurs ci-dessous sont celles par défaut.

Celles listées dans robot/tuning.py se règlent aussi depuis l'interface web : le code les lit
donc sous la forme `utils.KP` à chaque utilisation, jamais avec `from robot.utils import KP`.
"""

# Vitesses : fraction de la vitesse maximale des moteurs (0 à 1).
BASE_SPEED = 0.4  # sur la ligne
FORWARD_SPEED = 0.1  # à l'approche d'un croisement, ou quand la ligne est perdue
TURN_SPEED = 0.8  # rotation sur place
MAX_WHEEL_SPEED = 1.0  # au-delà, le PWM déborde sur l'Arduino et la roue ralentit

# Suivi de ligne : vitesse ajoutée à une roue et retirée à l'autre, ligne au bord de l'image.
KP = 0.15

# Durées, en secondes.
TURN_90_DURATION = 1.5  # rotation de 90°
TURN_180_DURATION = 3.0  # demi-tour
APPROACH_DURATION = 0.9  # approche du centre d'un croisement, à FORWARD_SPEED
RAMP_TIME_CONSTANT = 0.15  # constante de temps des accélérations et décélérations
LINE_LOST_TIMEOUT = 1.0  # cahier des charges : en suivi, arrêt si aucune ligne n'est vue pendant plus longtemps

# Écart à la consigne en dessous duquel la vitesse est considérée comme atteinte
# (une exponentielle n'atteint jamais sa consigne).
SPEED_EPSILON = 0.01

# Serveurs.
TELEMETRY_PORT = 8001  # doit correspondre à TELEMETRY_PORT dans frontend/utils.py
DASHBOARD_PORT = 8501
MAX_REQUEST_BYTES = 4096
