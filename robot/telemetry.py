"""Tableau de bord en direct : état du robot en JSON et page de la grille, servis par le robot.

    http://<IP_du_Pi>:8001/            grille en direct + image traitée (frontend/live.html)
    http://<IP_du_Pi>:8001/state.json  état courant du robot

Le serveur tourne dans un thread du programme du robot : une erreur ici ne doit jamais
l'empêcher de rouler.
"""
from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
import threading
from dataclasses import asdict, dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, List, Optional

from planning.direction import Direction
from planning.navigation import Mission, Position
from robot.states import Mode, RobotState

TELEMETRY_PORT = 8001    # doit correspondre à TELEMETRY_PORT dans frontend/app.py
DASHBOARD_PORT = 8501
FRONTEND_DIR = Path(__file__).resolve().parents[1] / "frontend"


@dataclass
class RobotStatus:
    mode: str
    state: str
    grid_nodes: int  # nombre d'intersections par côté
    start: Position
    destination: Position
    route: List[Position]  # chemin de l'étape en cours (aller ou retour)
    position: Optional[Position]  # dernier nœud atteint (None : pas encore arrivé au départ)
    next_node: Optional[Position]
    heading: str
    destination_reached: bool
    returned_home: bool
    line_detected: bool
    is_intersection: bool


def make_status(
    mode: Mode,
    state: RobotState,
    grid_nodes: int,
    mission: Mission,
    heading: Direction,
    line_detected: bool = False,
    is_intersection: bool = False,
) -> RobotStatus:
    return RobotStatus(
        mode=mode.name,
        state=state.name,
        grid_nodes=grid_nodes,
        start=mission.start,
        destination=mission.destination,
        route=list(mission.route),
        position=mission.position,
        next_node=mission.next_node,
        heading=heading.name,
        destination_reached=mission.destination_reached,
        returned_home=mission.returned_home,
        line_detected=line_detected,
        is_intersection=is_intersection,
    )


class TelemetryServer:
    def __init__(self, port: int = TELEMETRY_PORT) -> None:
        self.port = port
        self._lock = threading.Lock()
        self._state_json: Optional[bytes] = None
        page = (FRONTEND_DIR / "live.html").read_bytes()
        telemetry = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                path = self.path.split("?", 1)[0]
                if path == "/state.json":
                    body = telemetry._read_state()
                    if body is None:
                        self.send_error(503, "Robot pas encore démarré")
                    else:
                        self._send(body, "application/json")
                elif path in ("/", "/index.html"):
                    self._send(page, "text/html; charset=utf-8")
                else:
                    self.send_error(404)

            def _send(self, body: bytes, content_type: str) -> None:
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *args: Any) -> None:
                pass  # pas de log à chaque requête

        self._server = ThreadingHTTPServer(("0.0.0.0", port), Handler)  # OSError si port pris
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

    def _read_state(self) -> Optional[bytes]:
        with self._lock:
            return self._state_json

    def publish(self, status: RobotStatus) -> None:
        body = json.dumps(asdict(status)).encode()
        with self._lock:
            self._state_json = body

    def close(self) -> None:
        self._server.shutdown()
        self._server.server_close()


def start_telemetry(port: int = TELEMETRY_PORT) -> Optional[TelemetryServer]:
    try:
        server = TelemetryServer(port)
    except OSError as exc:
        print(f"[dashboard] tableau de bord désactivé (port {port} indisponible : {exc})")
        return None
    print(f"[dashboard] grille en direct : http://<IP_du_Pi>:{port}/")
    return server


def start_dashboard(port: int = DASHBOARD_PORT) -> Optional["subprocess.Popen[bytes]"]:
    """Lance l'interface Streamlit, accessible depuis le réseau sur http://<IP_du_Pi>:<port>."""
    if importlib.util.find_spec("streamlit") is None:
        print("[dashboard] Streamlit n'est pas installé (pip install -r frontend/requirements.txt) : "
              f"utiliser directement http://<IP_du_Pi>:{TELEMETRY_PORT}/")
        return None
    process = subprocess.Popen(
        [sys.executable, "-m", "streamlit", "run", "app.py",
         "--server.address", "0.0.0.0", "--server.port", str(port),
         "--server.headless", "true", "--browser.gatherUsageStats", "false"],
        cwd=str(FRONTEND_DIR),
    )
    print(f"[dashboard] interface : http://<IP_du_Pi>:{port}/")
    return process
