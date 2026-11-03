"""
Noeud de navigation du robot - EI ST5 VAC, equipe 10 (Stellantisky).

Tache T.4 du livrable 1. Defis indispensables couverts :
  - 3.3  Prendre la bonne direction a un croisement : a chaque croisement detecte,
         la consigne (gauche / droite / tout droit / demi-tour) est envoyee au controleur.
  - 4.2  Route saisie par l'operateur : suite de directions executee croisement par croisement.
  - 6.1  Systeme de coordonnees : quadrillage a noeuds numerotes (voir GridMap).
  - 6.2  Position : mise a jour a chaque croisement franchi + odometrie par encodeurs
         entre deux croisements (position cartesienne en cm).
  - 6.3  Point de depart et destination stockes dans un fichier JSON sur le Raspberry Pi
         et relus au redemarrage du programme.
En plus (peu couteux une fois la carte en graphe) : plus court chemin vers un noeud (4.3)
et retour au point de depart sur commande (6.4).

------------------------------------------------------------------------------------------
Communication ZeroMQ (remplace les topics ROS) - chaque message est un multipart
[topic (bytes), payload JSON (bytes)].

Ecoute (un seul socket SUB connecte a tous les publishers, cf. --connect) :
  corner          (cam)      {"x": float, "y": float} ou [x, y] ou {"detected": bool} ou null.
                             (0, 0) / null = pas de croisement (convention de cam.py).
  encoders        (controle) {"ticks": [fr, fl, rr, rl]} compteurs cumules des 4 encodeurs.
  maneuver_done   (controle) {"seq": int} manoeuvre de croisement terminee.
  nav_cmd         (IHM)      commandes operateur, champ "cmd" :
                               {"cmd": "set_pose", "from": a, "to": b}  robot pose sur l'arete a->b
                               {"cmd": "route", "directions": ["G", "T", "D", "U"]}  (ou "G T D")
                               {"cmd": "destination", "node": n}
                               {"cmd": "return_home"}
                               {"cmd": "go"} / {"cmd": "resume"}
                               {"cmd": "stop"}            (arret d'urgence cote navigation)
                               {"cmd": "clear"}           (oublie route et destination)

Publie (socket PUB lie sur --bind) :
  nav_action      (-> controle) {"action": "left"|"right"|"straight"|"u_turn"|"go"|"stop",
                                 "seq": int, "node": int|null}
                  Le controleur suit la ligne apres "go", s'arrete sur "stop", et execute la
                  manoeuvre demandee au croisement puis publie maneuver_done avec le meme seq.
  nav_status      (-> IHM, 5 Hz) etat complet : mode, pose, x/y, route restante, chemin...
  nav_event       (-> IHM) evenements ponctuels : crossing, arrived, error, warning...
------------------------------------------------------------------------------------------

Lancement (sur le Raspberry Pi) :
  python navigation.py --connect tcp://localhost:5556 --connect tcp://localhost:5557 \
                       --connect tcp://<IP_PC_IHM>:5558 --cols 5 --rows 5 --spacing 50
"""
from __future__ import annotations

import argparse
import json
import logging
import math
import os
import time
from collections import deque

import zmq

log = logging.getLogger("navigation")

# =========================================================================================
# Parametres (a recaler lors des essais sur piste)
# =========================================================================================

# --- Odometrie (moteurs Joy-IT COM-MOTOR06, cf. datasheet) ---
WHEEL_DIAMETER_CM = 6.5                      # roue omnidirectionnelle 65 mm
TICKS_PER_WHEEL_REV = 11 * 21.3              # 11 imp/tour moteur (front descendant de B) x reduction 21,3
CM_PER_TICK = math.pi * WHEEL_DIAMETER_CM / TICKS_PER_WHEEL_REV   # ~0.087 cm
ENCODER_SIGNS = (1, 1, 1, 1)                 # ordre [fr, fl, rr, rl] ; mettre -1 si un cote compte
                                             # negativement en marche avant
MAX_TICKS_JUMP = 5000                        # saut aberrant entre deux messages (reset Arduino) -> ignore
ODOMETRY_TIMEOUT_S = 1.0                     # au-dela, l'odometrie est consideree indisponible

# --- Carte par defaut (6.1) ---
DEFAULT_COLS = 5
DEFAULT_ROWS = 5
DEFAULT_SPACING_CM = 50.0                    # distance entre deux croisements : A MESURER sur la piste

# --- Detection des croisements (filtrage des detections de cam.py) ---
CROSSING_CONFIRM_FRAMES = 2                  # detections consecutives pour valider un croisement
CROSSING_CLEAR_FRAMES = 3                    # images sans croisement depuis le dernier, pour rearmer
CROSSING_AHEAD_CM = 15.0                     # distance robot -> centre du croisement lors de la detection
MIN_EDGE_FRACTION = 0.5                      # part de l'arete a parcourir avant d'accepter un croisement
MIN_CROSSING_INTERVAL_S = 1.5                # idem en temps, si l'odometrie est indisponible
MISSED_CROSSING_MARGIN = 0.5                 # > (1 + marge) x longueur d'arete sans croisement => manque

# --- Divers ---
MANEUVER_TIMEOUT_S = 6.0                     # si le controleur n'acquitte pas une manoeuvre
STATUS_PERIOD_S = 0.2                        # 5 Hz vers l'IHM
POLL_TIMEOUT_MS = 20
DEFAULT_STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "nav_state.json")

# --- Topics ZeroMQ ---
TOPIC_CORNER = "corner"
TOPIC_ENCODERS = "encoders"
TOPIC_MANEUVER_DONE = "maneuver_done"
TOPIC_NAV_CMD = "nav_cmd"
TOPIC_NAV_ACTION = "nav_action"
TOPIC_NAV_STATUS = "nav_status"
TOPIC_NAV_EVENT = "nav_event"

# =========================================================================================
# Directions
# =========================================================================================

LEFT, RIGHT, STRAIGHT, U_TURN = "left", "right", "straight", "u_turn"

_DIRECTION_ALIASES = {
    LEFT: LEFT, "l": LEFT, "g": LEFT, "gauche": LEFT,
    RIGHT: RIGHT, "r": RIGHT, "d": RIGHT, "droite": RIGHT,
    STRAIGHT: STRAIGHT, "s": STRAIGHT, "t": STRAIGHT, "tout droit": STRAIGHT,
    "tout_droit": STRAIGHT, "tout-droit": STRAIGHT, "toutdroit": STRAIGHT,
    U_TURN: U_TURN, "u": U_TURN, "uturn": U_TURN, "u-turn": U_TURN,
    "demi-tour": U_TURN, "demi_tour": U_TURN, "demitour": U_TURN, "demi tour": U_TURN,
}


def parse_direction(value) -> str:
    key = str(value).strip().lower()
    if key not in _DIRECTION_ALIASES:
        raise ValueError(f"direction inconnue : {value!r}")
    return _DIRECTION_ALIASES[key]


def parse_directions(value) -> list:
    """Accepte une liste ["G", "tout droit", ...] ou une chaine "G T D" / "G,T,D"."""
    if isinstance(value, str):
        value = [v for v in value.replace(",", " ").replace(";", " ").split() if v]
    if not isinstance(value, (list, tuple)):
        raise ValueError("la route doit etre une liste de directions")
    return [parse_direction(v) for v in value]


# Repere de la carte : x vers la droite, y vers le BAS (comme la carte dessinee, noeud 0 en haut
# a gauche). Dans ce repere, tourner a droite = sens horaire a l'ecran.
def turn_heading(heading: tuple, action: str) -> tuple:
    dx, dy = heading
    if action == STRAIGHT:
        return (dx, dy)
    if action == U_TURN:
        return (-dx, -dy)
    if action == RIGHT:
        return (-dy, dx)
    if action == LEFT:
        return (dy, -dx)
    raise ValueError(action)


def action_between(heading_in: tuple, heading_out: tuple) -> str:
    for action in (STRAIGHT, LEFT, RIGHT, U_TURN):
        if turn_heading(heading_in, action) == heading_out:
            return action
    raise ValueError(f"caps incompatibles {heading_in} -> {heading_out}")


# =========================================================================================
# Carte (defi 6.1)
# =========================================================================================

class GridMap:
    """Quadrillage cols x rows. Noeud n <-> (col, row) = (n % cols, n // cols).

        0 -- 1 -- 2 -- 3 -- 4        x (cm) = col * spacing
        |    |    |    |    |        y (cm) = row * spacing   (y vers le bas)
        5 -- 6 -- 7 -- 8 -- 9
        ...
    Des aretes peuvent etre retirees (piste incomplete, ou arete bloquee par un obstacle).
    """

    def __init__(self, cols: int, rows: int, spacing_cm: float, removed_edges=()):
        if cols < 1 or rows < 1 or spacing_cm <= 0:
            raise ValueError("carte invalide")
        self.cols = int(cols)
        self.rows = int(rows)
        self.spacing = float(spacing_cm)
        self.removed = set()
        for a, b in removed_edges:
            self.remove_edge(int(a), int(b))

    @classmethod
    def from_file(cls, path: str) -> "GridMap":
        """Fichier JSON : {"cols": 5, "rows": 5, "spacing_cm": 50, "removed_edges": [[1, 2]]}"""
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        return cls(data["cols"], data["rows"], data["spacing_cm"], data.get("removed_edges", []))

    @property
    def n_nodes(self) -> int:
        return self.cols * self.rows

    def is_node(self, n) -> bool:
        return isinstance(n, int) and not isinstance(n, bool) and 0 <= n < self.n_nodes

    def col_row(self, n: int) -> tuple:
        return n % self.cols, n // self.cols

    def coords_cm(self, n: int) -> tuple:
        c, r = self.col_row(n)
        return c * self.spacing, r * self.spacing

    def remove_edge(self, a: int, b: int):
        self.removed.add(frozenset((a, b)))

    def neighbor(self, n: int, heading: tuple):
        """Noeud atteint depuis n en suivant le cap (dx, dy), ou None si pas d'arete."""
        c, r = self.col_row(n)
        c2, r2 = c + heading[0], r + heading[1]
        if not (0 <= c2 < self.cols and 0 <= r2 < self.rows):
            return None
        m = r2 * self.cols + c2
        if frozenset((n, m)) in self.removed:
            return None
        return m

    def neighbors(self, n: int) -> list:
        out = []
        for h in ((1, 0), (0, 1), (-1, 0), (0, -1)):
            m = self.neighbor(n, h)
            if m is not None:
                out.append(m)
        return out

    def has_edge(self, a: int, b: int) -> bool:
        return self.is_node(a) and self.is_node(b) and b in self.neighbors(a)

    def heading(self, a: int, b: int) -> tuple:
        (ca, ra), (cb, rb) = self.col_row(a), self.col_row(b)
        h = (cb - ca, rb - ra)
        if abs(h[0]) + abs(h[1]) != 1:
            raise ValueError(f"{a} et {b} ne sont pas voisins")
        return h

    def edge_length(self, a: int, b: int) -> float:
        return self.spacing

    def shortest_path(self, src: int, dst: int):
        """Parcours en largeur (toutes les aretes ont la meme longueur). Liste de noeuds ou None."""
        if src == dst:
            return [src]
        parent = {src: None}
        queue = deque([src])
        while queue:
            n = queue.popleft()
            for m in self.neighbors(n):
                if m in parent:
                    continue
                parent[m] = n
                if m == dst:
                    path = [m]
                    while parent[path[-1]] is not None:
                        path.append(parent[path[-1]])
                    return path[::-1]
                queue.append(m)
        return None


# =========================================================================================
# Logique de navigation (independante de ZeroMQ, testable hors robot)
# =========================================================================================

MODE_IDLE = "idle"              # immobile, en attente d'une mission
MODE_RUNNING = "running"        # suivi de ligne vers le prochain croisement
MODE_MANEUVER = "maneuver"      # manoeuvre de croisement en cours (controleur)
MODE_PAUSED = "paused"          # arret demande par l'operateur
MODE_ARRIVED = "arrived"        # destination / fin de route atteinte


class Navigator:
    """Etat de navigation du robot.

    Pose = arete orientee (prev -> next) + distance parcourue depuis prev (cm).
    La route est une file de directions : route[0] est l'action a effectuer au noeud `next`.
    Les messages a envoyer sont empiles dans `outbox` sous forme (topic, payload).
    """

    def __init__(self, grid: GridMap, state_file: str | None = None, clock=time.monotonic):
        self.grid = grid
        self.state_file = state_file
        self.clock = clock
        self.outbox = []

        self.prev = None
        self.next = None
        self.dist_cm = 0.0
        self.at_node = False          # True : robot arrete devant le croisement `next`
        self.start = None             # defi 6.3
        self.destination = None       # defi 6.3
        self.route = deque()
        self.planned_nodes = []       # noeuds restants a visiter (affichage IHM)
        self.mode = MODE_IDLE
        self.last_error = None
        self.crossings_count = 0

        self._seq = 0
        self._maneuver_seq = None
        self._maneuver_deadline = 0.0
        self._seen_frames = 0
        self._clear_frames = CROSSING_CLEAR_FRAMES
        self._check_progress = False  # pas de contrainte de distance sur la premiere arete
        self._last_crossing_t = -math.inf
        self._last_odo_t = -math.inf
        self._last_ticks = None

        self.load()

    # ------------------------------------------------------------------ utilitaires
    @property
    def has_pose(self) -> bool:
        return self.prev is not None and self.next is not None

    def _edge_len(self) -> float:
        return self.grid.edge_length(self.prev, self.next)

    def _odometry_ok(self) -> bool:
        return self.clock() - self._last_odo_t < ODOMETRY_TIMEOUT_S

    def _emit_action(self, action: str, node=None):
        self._seq += 1
        self.outbox.append((TOPIC_NAV_ACTION, {"action": action, "seq": self._seq, "node": node}))
        return self._seq

    def _event(self, kind: str, **data):
        data["event"] = kind
        self.outbox.append((TOPIC_NAV_EVENT, data))
        if kind in ("error", "warning"):
            log.warning("%s : %s", kind, data.get("message"))
        else:
            log.info("%s %s", kind, {k: v for k, v in data.items() if k != "event"})

    def _error(self, message: str, stop: bool = True):
        self.last_error = message
        self._event("error", message=message)
        if stop:
            self._halt(MODE_IDLE)

    def _halt(self, mode: str):
        if self.mode in (MODE_RUNNING, MODE_MANEUVER):
            self._emit_action("stop", self.next)
        self.mode = mode
        self._maneuver_seq = None

    def _simulate(self, directions: list):
        """Noeuds visites en executant `directions` a partir de la pose courante, ou ValueError."""
        node = self.next
        heading = self.grid.heading(self.prev, self.next)
        nodes = [node]
        for i, action in enumerate(directions):
            heading = turn_heading(heading, action)
            nxt = self.grid.neighbor(node, heading)
            if nxt is None:
                raise ValueError(f"direction n°{i + 1} ({action}) impossible au noeud {node}")
            node = nxt
            nodes.append(node)
        return nodes

    def _directions_for_path(self, path: list) -> list:
        """Chemin de noeuds commencant par self.next -> liste de directions."""
        heading = self.grid.heading(self.prev, self.next)
        directions = []
        for a, b in zip(path, path[1:]):
            h = self.grid.heading(a, b)
            directions.append(action_between(heading, h))
            heading = h
        return directions

    # ------------------------------------------------------------------ position (6.2)
    def position_cm(self):
        """Position cartesienne estimee (x, y) en cm dans le repere de la carte."""
        if not self.has_pose:
            return None
        x0, y0 = self.grid.coords_cm(self.prev)
        dx, dy = self.grid.heading(self.prev, self.next)
        d = min(max(self.dist_cm, 0.0), self._edge_len())
        return (x0 + dx * d, y0 + dy * d)

    def current_node(self):
        """Noeud le plus proche sur l'arete courante."""
        if not self.has_pose:
            return None
        return self.next if self.dist_cm >= self._edge_len() / 2 else self.prev

    # ------------------------------------------------------------------ commandes operateur
    def handle_command(self, msg: dict):
        if not isinstance(msg, dict) or "cmd" not in msg:
            self._error(f"commande invalide : {msg!r}", stop=False)
            return
        cmd = str(msg["cmd"]).lower()
        try:
            if cmd == "set_pose":
                self.set_pose(int(msg["from"]), int(msg["to"]))
            elif cmd == "route":
                self.set_route(parse_directions(msg.get("directions", [])), autostart=msg.get("start", True))
            elif cmd == "destination":
                self.set_destination(int(msg["node"]), autostart=msg.get("start", True))
            elif cmd == "return_home":
                self.return_home(autostart=msg.get("start", True))
            elif cmd in ("go", "resume"):
                self.go()
            elif cmd == "stop":
                self.stop()
            elif cmd == "clear":
                self.clear_mission()
            else:
                self._error(f"commande inconnue : {cmd}", stop=False)
        except (KeyError, TypeError, ValueError) as exc:
            self._error(f"commande {cmd} refusee : {exc}", stop=False)

    def set_pose(self, prev: int, nxt: int):
        """Le robot est pose sur l'arete prev -> nxt, oriente vers nxt. prev devient le depart."""
        if not self.grid.has_edge(prev, nxt):
            raise ValueError(f"pas d'arete {prev} -> {nxt} sur la carte")
        self._halt(MODE_IDLE)
        self.prev, self.next = prev, nxt
        self.dist_cm = 0.0
        self.at_node = False
        self.start = prev
        self.route.clear()
        self.planned_nodes = []
        self.last_error = None
        self._check_progress = False
        self._clear_frames = CROSSING_CLEAR_FRAMES
        self._event("pose_set", prev=prev, next=nxt, start=prev)
        self.save()

    def set_route(self, directions: list, autostart: bool = True):
        """Defi 4.2 : suite de directions executee croisement par croisement."""
        if not self.has_pose:
            raise ValueError("position inconnue : envoyer d'abord set_pose")
        if self.mode == MODE_MANEUVER:
            raise ValueError("manoeuvre en cours, reessayer apres")
        nodes = self._simulate(directions)
        self.route = deque(directions)
        self.planned_nodes = nodes
        self.destination = nodes[-1]
        self.last_error = None
        self._event("route_set", directions=directions, nodes=nodes, destination=self.destination)
        self.save()
        if autostart:
            self.go()

    def set_destination(self, node: int, autostart: bool = True):
        """Destination = noeud de la carte ; itineraire = plus court chemin depuis la pose."""
        if not self.grid.is_node(node):
            raise ValueError(f"noeud {node} hors de la carte")
        self.destination = node
        self.save()
        if not self.has_pose:
            raise ValueError("position inconnue : envoyer d'abord set_pose (destination memorisee)")
        self._plan_to_destination()
        if autostart:
            self.go()

    def return_home(self, autostart: bool = True):
        if self.start is None:
            raise ValueError("point de depart inconnu")
        self.set_destination(self.start, autostart)

    def _plan_to_destination(self):
        if self.mode == MODE_MANEUVER:
            raise ValueError("manoeuvre en cours, reessayer apres")
        path = self.grid.shortest_path(self.next, self.destination)
        if path is None:
            raise ValueError(f"aucun chemin vers le noeud {self.destination}")
        self.route = deque(self._directions_for_path(path))
        self.planned_nodes = path
        self.last_error = None
        self._event("route_set", directions=list(self.route), nodes=path, destination=self.destination)
        self.save()

    def go(self):
        """Demarre / reprend la mission en cours."""
        if not self.has_pose:
            raise ValueError("position inconnue : envoyer d'abord set_pose")
        if self.mode in (MODE_RUNNING, MODE_MANEUVER):
            return
        if not self.route and self.destination is not None and not (self.at_node and self.next == self.destination):
            self._plan_to_destination()
        self._event("started", route=list(self.route), destination=self.destination)
        if self.at_node:
            # Robot deja arrete devant le croisement `next` : on y execute directement l'action.
            self.mode = MODE_RUNNING
            self._execute_next_action()
            return
        self.mode = MODE_RUNNING
        self._emit_action("go", self.next)

    def stop(self):
        """Arret operateur (l'arret d'urgence materiel reste gere par le controle / l'Arduino).
        Un "stop" est toujours envoye au controleur, meme si le robot semble deja arrete."""
        self.mode = MODE_PAUSED
        self._maneuver_seq = None
        self._emit_action("stop", self.next)
        self._event("stopped")
        self.save()

    def clear_mission(self):
        self._halt(MODE_IDLE)
        self.route.clear()
        self.planned_nodes = []
        self.destination = None
        self._event("mission_cleared")
        self.save()

    # ------------------------------------------------------------------ capteurs
    def on_corner(self, detected: bool):
        """Detection de croisement par la camera (une fois par image)."""
        if detected:
            self._seen_frames += 1
        else:
            self._seen_frames = 0
            self._clear_frames += 1
            return
        if self.mode != MODE_RUNNING or self.at_node:
            return
        if self._seen_frames < CROSSING_CONFIRM_FRAMES or self._clear_frames < CROSSING_CLEAR_FRAMES:
            return
        if self._check_progress:
            if self._odometry_ok():
                if self.dist_cm < MIN_EDGE_FRACTION * self._edge_len():
                    return   # trop tot : probablement le croisement qu'on vient de quitter
            elif self.clock() - self._last_crossing_t < MIN_CROSSING_INTERVAL_S:
                return
        self._handle_crossing(detected=True)

    def on_encoders(self, ticks):
        """Compteurs cumules des 4 encodeurs -> distance parcourue sur l'arete (6.2)."""
        if len(ticks) != 4:
            raise ValueError("4 valeurs d'encodeurs attendues")
        ticks = [int(t) * s for t, s in zip(ticks, ENCODER_SIGNS)]
        last, self._last_ticks = self._last_ticks, ticks
        if last is None:
            return
        deltas = [t - l for t, l in zip(ticks, last)]
        if any(abs(d) > MAX_TICKS_JUMP for d in deltas):
            log.warning("saut d'encodeurs ignore : %s", deltas)
            return
        self._last_odo_t = self.clock()
        if self.mode == MODE_RUNNING and not self.at_node:
            # Moyenne des 4 roues : en ligne droite toutes avancent, en rotation sur place
            # les deux cotes se compensent.
            self.dist_cm += sum(deltas) / 4.0 * CM_PER_TICK

    def on_maneuver_done(self, seq=None):
        if self.mode != MODE_MANEUVER:
            return
        if seq is not None and self._maneuver_seq is not None and int(seq) != self._maneuver_seq:
            return
        self._finish_maneuver()

    def _finish_maneuver(self):
        self.mode = MODE_RUNNING
        self._maneuver_seq = None
        self.dist_cm = 0.0     # fin de virage : robot au centre du croisement, sur la nouvelle arete
        self._event("maneuver_done", prev=self.prev, next=self.next)

    # ------------------------------------------------------------------ croisements (3.3, 6.2)
    def _handle_crossing(self, detected: bool, overshoot_cm: float = 0.0):
        """Le robot atteint le croisement `next` : mise a jour de la position puis action."""
        node = self.next
        self.crossings_count += 1
        self._last_crossing_t = self.clock()
        self._clear_frames = 0
        self._seen_frames = 0
        self._check_progress = True
        self.at_node = True
        if self.planned_nodes and self.planned_nodes[0] == node:
            self.planned_nodes = self.planned_nodes[1:]
        self._event("crossing", node=node, detected=detected, position_cm=self.grid.coords_cm(node))
        self._execute_next_action(overshoot_cm)

    def _execute_next_action(self, overshoot_cm: float = 0.0):
        """Robot au croisement `next` : execute route[0], ou s'arrete si la route est finie."""
        node = self.next
        now = self.clock()
        if not self.route:
            self.dist_cm = self._edge_len() - CROSSING_AHEAD_CM
            if self.destination is not None and node == self.destination:
                self._halt(MODE_ARRIVED)
                self._event("arrived", node=node, start=self.start)
            else:
                self._halt(MODE_IDLE)
                self._event("route_finished", node=node, destination=self.destination)
            self.save()
            return

        action = self.route[0]
        heading = turn_heading(self.grid.heading(self.prev, node), action)
        nxt = self.grid.neighbor(node, heading)
        if nxt is None:   # ne devrait pas arriver (route validee), sauf carte modifiee entre-temps
            self.dist_cm = self._edge_len() - CROSSING_AHEAD_CM
            self._error(f"action {action} impossible au noeud {node}")
            self.save()
            return
        self.route.popleft()
        self.prev, self.next = node, nxt
        self.at_node = False
        seq = self._emit_action(action, node)
        if action == STRAIGHT:
            self.mode = MODE_RUNNING
            self.dist_cm = overshoot_cm if overshoot_cm else -CROSSING_AHEAD_CM
        else:
            self.mode = MODE_MANEUVER
            self._maneuver_seq = seq
            self._maneuver_deadline = now + MANEUVER_TIMEOUT_S
            self.dist_cm = 0.0
        self.save()

    def tick(self):
        """A appeler regulierement : timeouts de manoeuvre et croisements manques."""
        now = self.clock()
        if self.mode == MODE_MANEUVER and now > self._maneuver_deadline:
            self._event("warning", message="manoeuvre non acquittee par le controleur (timeout)")
            self._finish_maneuver()
        if (self.mode == MODE_RUNNING and not self.at_node and self._check_progress
                and self._odometry_ok()
                and self.dist_cm > (1 + MISSED_CROSSING_MARGIN) * self._edge_len()):
            overshoot = self.dist_cm - self._edge_len()
            if self.route and self.route[0] == STRAIGHT:
                # Croisement manque mais on devait aller tout droit : on corrige la position.
                self._event("warning", message=f"croisement {self.next} manque, position corrigee par odometrie")
                self._handle_crossing(detected=False, overshoot_cm=overshoot)
            else:
                # Trop tard pour tourner : on s'arrete plutot que de partir au hasard.
                node = self.next
                beyond = self.grid.neighbor(node, self.grid.heading(self.prev, node))
                if beyond is not None:
                    self.prev, self.next, self.dist_cm = node, beyond, overshoot
                self._error(f"croisement {node} manque avant un virage ou l'arrivee : robot arrete, "
                            f"repositionner avec set_pose")
                self.save()

    # ------------------------------------------------------------------ IHM
    def status(self) -> dict:
        pos = self.position_cm()
        return {
            "mode": self.mode,
            "prev": self.prev,
            "next": self.next,
            "at_node": self.at_node,
            "current_node": self.current_node(),
            "dist_cm": round(self.dist_cm, 1),
            "x_cm": None if pos is None else round(pos[0], 1),
            "y_cm": None if pos is None else round(pos[1], 1),
            "heading": None if not self.has_pose else self.grid.heading(self.prev, self.next),
            "start": self.start,
            "destination": self.destination,
            "route": list(self.route),
            "planned_nodes": self.planned_nodes,
            "crossings": self.crossings_count,
            "odometry_ok": self._odometry_ok(),
            "last_error": self.last_error,
            "map": {"cols": self.grid.cols, "rows": self.grid.rows, "spacing_cm": self.grid.spacing},
        }

    # ------------------------------------------------------------------ persistance (6.3)
    def save(self):
        if not self.state_file:
            return
        data = {
            "start": self.start,
            "destination": self.destination,
            "prev": self.prev,
            "next": self.next,
            "at_node": self.at_node,
            "route": list(self.route),
            "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        tmp = self.state_file + ".tmp"
        try:
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            os.replace(tmp, self.state_file)   # ecriture atomique (coupure batterie)
        except OSError as exc:
            log.error("sauvegarde impossible (%s) : %s", self.state_file, exc)

    def load(self):
        """Relit depart, destination, pose et route. Le robot reste immobile (mode idle)."""
        if not self.state_file or not os.path.exists(self.state_file):
            return
        try:
            with open(self.state_file, encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, ValueError) as exc:
            log.error("fichier d'etat illisible (%s) : %s", self.state_file, exc)
            return
        g = self.grid
        if g.is_node(data.get("start")):
            self.start = data["start"]
        if g.is_node(data.get("destination")):
            self.destination = data["destination"]
        prev, nxt = data.get("prev"), data.get("next")
        if g.is_node(prev) and g.is_node(nxt) and g.has_edge(prev, nxt):
            self.prev, self.next = prev, nxt
            self.at_node = bool(data.get("at_node", False))
            if self.at_node:
                self.dist_cm = self._edge_len() - CROSSING_AHEAD_CM
            try:
                route = [parse_direction(d) for d in data.get("route", [])]
                self.planned_nodes = self._simulate(route)
                self.route = deque(route)
            except ValueError:
                self.route = deque()
        log.info("etat recharge : depart=%s destination=%s pose=%s->%s route=%s",
                 self.start, self.destination, self.prev, self.next, list(self.route))


# =========================================================================================
# Decodage des messages
# =========================================================================================

def corner_detected(payload) -> bool:
    if payload is None:
        return False
    if isinstance(payload, dict):
        if "detected" in payload:
            return bool(payload["detected"])
        x, y = payload.get("x"), payload.get("y")
    elif isinstance(payload, (list, tuple)) and len(payload) >= 2:
        x, y = payload[0], payload[1]
    else:
        return False
    if x is None or y is None:
        return False
    return not (float(x) == 0.0 and float(y) == 0.0)   # (0, 0) = rien detecte (cf. cam.py)


def encoder_ticks(payload):
    if isinstance(payload, dict):
        payload = payload.get("ticks")
    if not isinstance(payload, (list, tuple)) or len(payload) != 4:
        raise ValueError(f"message encodeurs invalide : {payload!r}")
    return payload


# =========================================================================================
# Noeud ZeroMQ
# =========================================================================================

class NavigationNode:

    def __init__(self, navigator: Navigator, bind: str, connect: list):
        self.nav = navigator
        self.ctx = zmq.Context.instance()

        self.pub = self.ctx.socket(zmq.PUB)
        self.pub.setsockopt(zmq.LINGER, 500)
        self.pub.bind(bind)

        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.setsockopt(zmq.LINGER, 0)
        for endpoint in connect:
            self.sub.connect(endpoint)
        for topic in (TOPIC_CORNER, TOPIC_ENCODERS, TOPIC_MANEUVER_DONE, TOPIC_NAV_CMD):
            self.sub.setsockopt(zmq.SUBSCRIBE, topic.encode())

        self.poller = zmq.Poller()
        self.poller.register(self.sub, zmq.POLLIN)
        self._next_status = 0.0
        log.info("navigation : PUB sur %s, SUB connecte a %s", bind, connect)

    def publish(self, topic: str, payload):
        self.pub.send_multipart([topic.encode(), json.dumps(payload).encode()])

    def flush(self):
        while self.nav.outbox:
            self.publish(*self.nav.outbox.pop(0))

    def dispatch(self, topic: str, payload):
        if topic == TOPIC_CORNER:
            self.nav.on_corner(corner_detected(payload))
        elif topic == TOPIC_ENCODERS:
            self.nav.on_encoders(encoder_ticks(payload))
        elif topic == TOPIC_MANEUVER_DONE:
            self.nav.on_maneuver_done(payload.get("seq") if isinstance(payload, dict) else None)
        elif topic == TOPIC_NAV_CMD:
            self.nav.handle_command(payload)

    def receive_all(self):
        while True:
            try:
                frames = self.sub.recv_multipart(zmq.NOBLOCK)
            except zmq.Again:
                return
            if len(frames) != 2:
                log.warning("message ignore (%d trames)", len(frames))
                continue
            topic = frames[0].decode(errors="replace")
            try:
                self.dispatch(topic, json.loads(frames[1].decode()))
            except (ValueError, TypeError, AttributeError) as exc:
                log.warning("message %s ignore : %s", topic, exc)

    def spin_once(self):
        events = dict(self.poller.poll(POLL_TIMEOUT_MS))
        if self.sub in events:
            self.receive_all()
        self.nav.tick()
        self.flush()
        now = time.monotonic()
        if now >= self._next_status:
            self._next_status = now + STATUS_PERIOD_S
            self.publish(TOPIC_NAV_STATUS, self.nav.status())

    def spin(self):
        try:
            while True:
                self.spin_once()
        except KeyboardInterrupt:
            log.info("arret demande (Ctrl+C)")
        finally:
            self.nav.stop()
            self.flush()
            time.sleep(0.1)
            self.sub.close()
            self.pub.close()


# =========================================================================================
# Point d'entree
# =========================================================================================

def parse_args(argv=None):
    p = argparse.ArgumentParser(description="Noeud de navigation (ZeroMQ)")
    p.add_argument("--bind", default="tcp://*:5560", help="adresse du PUB de navigation")
    p.add_argument("--connect", action="append", default=None,
                   help="publisher a ecouter (repetable) : cam, controle, IHM")
    p.add_argument("--map", help="fichier JSON de carte (sinon quadrillage --cols x --rows)")
    p.add_argument("--cols", type=int, default=DEFAULT_COLS)
    p.add_argument("--rows", type=int, default=DEFAULT_ROWS)
    p.add_argument("--spacing", type=float, default=DEFAULT_SPACING_CM, help="cm entre deux croisements")
    p.add_argument("--state-file", default=DEFAULT_STATE_FILE, help="fichier de persistance (defi 6.3)")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args(argv)
    if args.connect is None:
        args.connect = ["tcp://localhost:5556", "tcp://localhost:5557", "tcp://localhost:5558"]
    return args


def main(argv=None):
    args = parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s [%(name)s] %(levelname)s %(message)s")
    grid = GridMap.from_file(args.map) if args.map else GridMap(args.cols, args.rows, args.spacing)
    navigator = Navigator(grid, state_file=args.state_file)
    NavigationNode(navigator, args.bind, args.connect).spin()


if __name__ == '__main__':
    main()
