"""Serveur HTTP du robot pour l'interface (état en JSON, page de commande, ordres).

    http://<IP_du_Pi>:8001/            choix du mode, grille en direct et image traitée
    http://<IP_du_Pi>:8001/state.json  état courant du robot
    POST /mission                      {"start": [x, y], "destination": [x, y], "heading": "SOUTH"} : livraison
                                       {"mode": "HUIT"} : suivi de ligne sur le parcours en 8
    POST /stop                         coupe les moteurs immédiatement
    POST /map/reset                    réinitialise la carte : plus aucune arête bloquée
    GET  /settings                     réglages et valeurs courantes
    POST /settings                     {"values": {"KP": 0.2}}, appliqué tout de suite
    POST /settings/reset               {"keys": ["KP"]}, ou {} pour tout réinitialiser

Le serveur tourne dans un thread à part. Il dépose les ordres et la boucle du robot vient les lire.
Il n'y a pas d'authentification, à utiliser seulement sur un réseau de confiance.
"""
from __future__ import annotations

import json
import os
import socket
import sys
import threading
import webbrowser
from dataclasses import asdict, dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

from planning.direction import Direction
from planning.navigation import Mission, Position
from planning.utils import GRID_SIZE
from robot import tuning
from robot.obstacle import ObstacleState, ObstacleStatus
from robot.utils import MAX_REQUEST_BYTES, TELEMETRY_PORT
from robot.states import Mode, RobotState

FRONTEND_DIR = Path(__file__).resolve().parents[1] / "frontend"


@dataclass(frozen=True)
class MissionRequest:
    start: Position
    destination: Position
    heading: Direction


@dataclass(frozen=True)
class FigureEightRequest:
    """Ordre pour lancer le parcours en 8, il n'a pas de paramètres."""


Order = Union[MissionRequest, FigureEightRequest]


def _parse_node(value: Any, name: str, grid_nodes: int) -> Position:
    valid = (isinstance(value, list) and len(value) == 2
             and all(type(v) is int and 0 <= v < grid_nodes for v in value))
    if not valid:
        raise ValueError(f"{name} : position hors de la grille.")
    return value[0], value[1]


def _parse_object(body: bytes) -> Dict[str, Any]:
    try:
        data = json.loads(body.decode("utf-8"))
    except (UnicodeDecodeError, ValueError):
        raise ValueError("Requête illisible.") from None
    if not isinstance(data, dict):
        raise ValueError("Requête illisible.")
    return data


def apply_settings(path: str, body: bytes) -> None:
    """Applique un changement de réglages. ValueError avec le message à afficher si ça ne va pas."""
    data = _parse_object(body)
    if path == "/settings":
        values = data.get("values")
        if not isinstance(values, dict):
            raise ValueError("Requête illisible.")
        tuning.update(values)
    else:
        keys = data.get("keys")
        if keys is not None and not (isinstance(keys, list) and all(isinstance(key, str) for key in keys)):
            raise ValueError("Requête illisible.")
        tuning.reset(keys)


def parse_mission(body: bytes, grid_nodes: int) -> Order:
    """Vérifie un ordre reçu de l'interface. ValueError avec le message à afficher si ça ne va pas."""
    data = _parse_object(body)
    mode = data.get("mode", Mode.GRID.name)
    if mode == Mode.HUIT.name:
        return FigureEightRequest()
    if mode != Mode.GRID.name:
        raise ValueError("Mode inconnu.")
    start = _parse_node(data.get("start"), "Départ", grid_nodes)
    destination = _parse_node(data.get("destination"), "Arrivée", grid_nodes)
    if start == destination:
        raise ValueError("Le départ et l'arrivée doivent être différents.")
    heading = data.get("heading")
    if not isinstance(heading, str) or heading not in Direction.__members__:
        raise ValueError("Orientation inconnue.")
    return MissionRequest(start, destination, Direction[heading])


@dataclass
class RobotStatus:
    mode: str
    state: str
    grid_nodes: int  # nombre d'intersections par côté
    start: Optional[Position]  # None si aucune mission lancée
    destination: Optional[Position]
    route: List[Position]  # chemin de l'étape en cours (aller ou retour)
    position: Optional[Position]  # dernier noeud atteint, None si aucune mission lancée
    next_node: Optional[Position]
    heading: Optional[str]
    destination_reached: bool
    returned_home: bool
    line_detected: bool
    is_intersection: bool
    line_lost: bool  # dernière mission arrêtée car ligne perdue
    obstacle_state: str
    obstacle_cm: Optional[int]  # None si pas de mesure valide
    blocked_edges: List[Tuple[Position, Position]]  # arêtes retirées du graphe
    no_route: bool  # dernière mission arrêtée car plus de chemin


def make_status(
    mode: Mode,
    state: RobotState,
    grid_nodes: int,
    mission: Optional[Mission],
    line_detected: bool = False,
    is_intersection: bool = False,
    line_lost: bool = False,
    obstacle: ObstacleStatus = ObstacleStatus(ObstacleState.NO_MEASURE, None),
    blocked_edges: Optional[List[Tuple[Position, Position]]] = None,
) -> RobotStatus:
    return RobotStatus(
        mode=mode.name,
        state=state.name,
        grid_nodes=grid_nodes,
        start=mission.start if mission else None,
        destination=mission.destination if mission else None,
        route=list(mission.route) if mission else [],
        position=mission.position if mission else None,
        next_node=mission.next_node if mission else None,
        heading=mission.heading.name if mission and mission.heading else None,
        destination_reached=mission.destination_reached if mission else False,
        returned_home=mission.returned_home if mission else False,
        line_detected=line_detected,
        is_intersection=is_intersection,
        line_lost=line_lost,
        obstacle_state=obstacle.state.name,
        obstacle_cm=obstacle.distance_cm,
        blocked_edges=blocked_edges or [],
        no_route=mission.no_route if mission else False,
    )


class TelemetryServer:
    def __init__(self, port: int = TELEMETRY_PORT, grid_nodes: int = GRID_SIZE + 1) -> None:
        self.port = port
        self._lock = threading.Lock()
        self._state_json: Optional[bytes] = None
        self._pending: Optional[Order] = None
        self._map_reset = False
        self._busy = False  # une mission est en cours
        self._stop_requested = threading.Event()
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
                elif path == "/settings":
                    self._send_settings()
                elif path in ("/", "/index.html"):
                    self._send(page, "text/html; charset=utf-8")
                else:
                    self.send_error(404)

            def do_POST(self) -> None:
                length = int(self.headers.get("Content-Length") or 0)
                if not 0 <= length <= MAX_REQUEST_BYTES:
                    self._reply(413, "Requête trop longue.")
                    return
                # À lire avant de répondre : sous Windows, fermer le socket avec des données
                # non lues envoie un RST et le client perd la réponse
                body = self.rfile.read(length)
                # On exige du JSON pour qu'un autre site ne puisse pas lancer le robot depuis le navigateur
                if self.headers.get("Content-Type", "").split(";")[0].strip() != "application/json":
                    self._reply(415, "Requête JSON attendue.")
                    return
                path = self.path.split("?", 1)[0]
                if path == "/mission":
                    try:
                        request = parse_mission(body, grid_nodes)
                    except ValueError as exc:
                        self._reply(400, str(exc))
                        return
                    if telemetry._submit_mission(request):
                        self._reply(200)
                    else:
                        self._reply(409, "Une mission est déjà en cours.")
                elif path == "/stop":
                    telemetry._request_stop()
                    self._reply(200)
                elif path == "/map/reset":
                    if telemetry._request_map_reset():
                        self._reply(200)
                    else:
                        self._reply(409, "Une mission est en cours.")
                elif path in ("/settings", "/settings/reset"):
                    try:
                        apply_settings(path, body)
                    except ValueError as exc:
                        self._reply(400, str(exc))
                        return
                    self._send_settings()
                else:
                    self.send_error(404)

            def _send_settings(self) -> None:
                self._send(json.dumps({"settings": tuning.describe()}).encode(), "application/json")

            def _reply(self, status: int, error: Optional[str] = None) -> None:
                body = json.dumps({"error": error}).encode()
                self._send(body, "application/json", status)

            def _send(self, body: bytes, content_type: str, status: int = 200) -> None:
                self.send_response(status)
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

    def _submit_mission(self, request: Order) -> bool:
        with self._lock:
            if self._busy or self._pending is not None:
                return False
            self._pending = request
            return True

    def _request_map_reset(self) -> bool:
        with self._lock:
            if self._busy or self._pending is not None:
                return False
            self._map_reset = True
            return True

    def take_map_reset(self) -> bool:
        """Renvoie True une seule fois après une demande de réinitialisation de la carte."""
        with self._lock:
            requested, self._map_reset = self._map_reset, False
            return requested

    def _request_stop(self) -> None:
        with self._lock:
            self._pending = None
            if self._busy:
                self._stop_requested.set()

    def take_mission(self) -> Optional[Order]:
        """Récupère l'ordre en attente (None s'il n'y en a pas) et marque la mission comme lancée."""
        with self._lock:
            request, self._pending = self._pending, None
            if request is not None:
                self._busy = True
                self._stop_requested.clear()
            return request

    def mission_finished(self) -> None:
        with self._lock:
            self._busy = False
            self._stop_requested.clear()

    @property
    def stop_requested(self) -> bool:
        return self._stop_requested.is_set()

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
        print(f"[interface] désactivée (port {port} indisponible : {exc})")
        return None
    return server


def local_ip() -> str:
    """Adresse IP de la machine sur le réseau (connect en UDP n'envoie aucun paquet)."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect(("10.255.255.255", 1))
            return str(probe.getsockname()[0])
    except OSError:
        return "<IP_du_Pi>"


def _has_display() -> bool:
    # En SSH sur le Pi, webbrowser lance un navigateur texte qui bloque le terminal
    return os.name == "nt" or sys.platform == "darwin" or bool(
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def open_interface(port: int = TELEMETRY_PORT) -> None:
    """Affiche l'adresse de l'interface et l'ouvre dans le navigateur si la machine a un écran."""
    print(f"[interface] http://{local_ip()}:{port}/")
    if _has_display():
        # Dans un thread car certains navigateurs bloquent jusqu'à leur fermeture
        threading.Thread(target=webbrowser.open, args=(f"http://localhost:{port}/",), daemon=True).start()
