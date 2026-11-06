"""Constantes des liaisons avec le matériel (valeurs par défaut).

Celles qui sont dans robot/tuning.py se règlent aussi depuis l'interface web.
"""

# Boucle du robot
TIME_STEP_MS = 64  # période de la boucle (dans Webots elle est fixée au démarrage)

# Liaison série avec l'Arduino
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
SERIAL_TIMEOUT = 0.1  # secondes
ARDUINO_BOOT_DELAY = 2.0  # secondes

# Capteur ultrason
ULTRASOUND_MAX_CM = 200  # ce que renvoie l'Arduino quand il ne voit rien (ULTRASON_code dans Commandes.cpp)

# Pour passer d'une vitesse (fraction de la vitesse max) à une commande moteur
PWM_MAX = 255
WEBOTS_SPEED_SCALE = 10

# Webots
WEBOTS_CAMERA_NAME = "PiCam"
