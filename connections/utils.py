"""Constantes des liaisons avec le matériel. Les valeurs ci-dessous sont celles par défaut.

Celles listées dans robot/tuning.py se règlent aussi depuis l'interface web.
"""

# Boucle du robot.
TIME_STEP_MS = 64  # période de la boucle ; figée au démarrage dans Webots

# Liaison série avec l'Arduino.
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
SERIAL_TIMEOUT = 0.1  # secondes
ARDUINO_BOOT_DELAY = 2.0  # secondes

# Conversion d'une vitesse (fraction de la vitesse maximale) en commande moteur.
PWM_MAX = 255
WEBOTS_SPEED_SCALE = 10

# Webots.
WEBOTS_CAMERA_NAME = "PiCam"
