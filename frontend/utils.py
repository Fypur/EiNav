"""Constantes de l'interface Streamlit."""
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
MAP_FILE = BASE_DIR / "carte.json"
LINE_FILE = BASE_DIR / "ligne_blanche.json"

MODE_NAV = "Navigation sur le maillage"
MODE_LINE = "Suivi de ligne blanche"

CAMERA_STREAM_PORT = 8000  # doit correspondre à STREAM_PORT dans vision/utils.py
TELEMETRY_PORT = 8001  # doit correspondre à TELEMETRY_PORT dans robot/utils.py
LIVE_PAGE_HEIGHT = 1000  # hauteur (px) de la page du robot ; au-delà elle défile, bouton Arrêter toujours visible
