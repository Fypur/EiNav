"""Navigation avec le vrai traitement d'image : images rendues depuis la position du robot, virages imprécis."""
from __future__ import annotations

import math
from typing import Any, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import pytest

from connections.base import RobotConnection
from planning.direction import Direction
from planning.navigation import Grid, Mission
from robot import tuning, utils
from robot.state_machine import run_state_machine
from vision import utils as vision_utils
from vision.image_processor import ImageProcessor

SPACING = 60.0  # cm entre deux croisements
GAIN = 150.0  # cm/s à pleine vitesse
LINE_WIDTH = 3.0  # cm
ANGLE = {Direction.NORTH: 0.0, Direction.EAST: 90.0, Direction.SOUTH: 180.0, Direction.WEST: 270.0}
ROWS, COLS = np.mgrid[0:128, 0:160]


# --- Reconnaissance d'un croisement ------------------------------------------------------------

def _frame(*polygons: Sequence[Sequence[Tuple[int, int]]]) -> np.ndarray:
    """Polygones blancs sur un sol sombre, en coordonnées de l'image 160 x 128."""
    frame = np.full((128, 160, 3), 40, np.uint8)
    for polygon in polygons:
        cv2.fillPoly(frame, [np.array(polygon, np.int32)], (220, 220, 220))
    return frame


def _intersection(frame: np.ndarray) -> bool:
    return ImageProcessor().process_frame(frame).is_intersection


# Ligne en biais passant par un coin de la zone analysée, comme sur les images réelles capturées.
SLANTED_THROUGH_CORNER = [(35, 76), (55, 76), (15, 127), (-5, 127)]
HUGGING_THE_EDGE = [(0, 76), (22, 76), (22, 127), (0, 127)]
CENTERED_LINE = [(70, 0), (90, 0), (90, 127), (70, 127)]


def test_ligne_en_biais_par_un_coin_n_est_pas_un_croisement() -> None:
    assert not _intersection(_frame(SLANTED_THROUGH_CORNER))


def test_ligne_le_long_du_bord_n_est_pas_un_croisement() -> None:
    assert not _intersection(_frame(HUGGING_THE_EDGE))


def test_ligne_centree_n_est_pas_un_croisement() -> None:
    assert not _intersection(_frame(CENTERED_LINE))


@pytest.mark.parametrize("branch", [
    [(0, 95), (160, 95), (160, 110), (0, 110)],  # croix
    [(80, 95), (160, 95), (160, 110), (80, 110)],  # T vers la droite
    [(0, 95), (80, 95), (80, 110), (0, 110)],  # T vers la gauche
])
def test_vrais_croisements_toujours_reconnus(branch: List[Tuple[int, int]]) -> None:
    assert _intersection(_frame(CENTERED_LINE, branch))


def test_t_vers_le_petit_cote_avec_la_ligne_decentree() -> None:
    """Ligne décalée vers le côté de la branche : la bande reste assez large, ligne comprise."""
    line = [(45, 0), (65, 0), (65, 127), (45, 127)]
    assert _intersection(_frame(line, [(0, 95), (65, 95), (65, 110), (0, 110)]))


def test_t_en_bord_de_grille_avec_la_ligne_decalee_vers_sa_branche() -> None:
    """Ne pas le voir décalerait le robot d'un nœud pour le reste de la mission."""
    line = [(100, 0), (120, 0), (120, 127), (100, 127)]
    assert _intersection(_frame(line, [(100, 95), (160, 95), (160, 110), (100, 110)]))


def test_seuil_reglable_depuis_l_interface() -> None:
    frame = _frame(CENTERED_LINE, [(80, 95), (160, 95), (160, 110), (80, 110)])
    assert _intersection(frame)
    tuning.update({"INTERSECTION_MIN_WIDTH_RATIO": 1.0})
    assert vision_utils.INTERSECTION_MIN_WIDTH_RATIO == 1.0
    assert not _intersection(frame)


# --- Contournement sur la grille rendue -----------------------------------------------------------

class RenderedWorld(RobotConnection):
    """Le robot se déplace selon ses roues ; la caméra rend la grille vue depuis sa position réelle."""

    def __init__(self, start: Tuple[int, int], heading: Direction, obstacles: Sequence[Tuple[float, float]],
                 turn_error: float = 1.0, max_steps: int = 1500) -> None:
        self.x, self.y = start[0] * SPACING, start[1] * SPACING
        self.angle = ANGLE[heading]
        self.obstacles = list(obstacles)
        self.turn_error = turn_error  # rotation réelle / rotation attendue, sur place
        self.dt, self.t, self.steps, self.max_steps = 0.064, 0.0, 0, max_steps
        self.left = self.right = 0.0
        self.deg_per_second = 180.0 / (utils.TURN_SPEED * utils.TURN_180_DURATION)
        reach = utils.FORWARD_SPEED * utils.APPROACH_DURATION * GAIN
        self.near = 5.0
        self.cm_per_pixel = (reach - self.near) / (128 - int(128 * vision_utils.ROI_TOP_RATIO))

    def get_time(self) -> float:
        return self.t

    def step(self) -> bool:
        self.t += self.dt
        self.steps += 1
        linear, angular = (self.left + self.right) / 2, (self.left - self.right) / 2
        spin = self.turn_error if abs(linear) < 1e-9 else 3.5  # le suivi de ligne dirige mieux qu'à l'arrêt
        self.angle = (self.angle + angular * self.deg_per_second * spin * self.dt) % 360
        radians = math.radians(self.angle)
        self.x += linear * GAIN * self.dt * math.sin(radians)
        self.y -= linear * GAIN * self.dt * math.cos(radians)
        return self.steps < self.max_steps

    def get_frame(self) -> np.ndarray:
        radians = math.radians(self.angle)
        forward = (math.sin(radians), -math.cos(radians))
        right = (math.cos(radians), math.sin(radians))
        along = self.near + (127 - ROWS) * self.cm_per_pixel
        side = (COLS - 79.5) * self.cm_per_pixel
        gx = (self.x + along * forward[0] + side * right[0]) / SPACING
        gy = (self.y + along * forward[1] + side * right[1]) / SPACING
        half = LINE_WIDTH / 2 / SPACING
        inside = (gx > -half) & (gx < 4 + half) & (gy > -half) & (gy < 4 + half)
        on_line = inside & ((np.abs(gx - np.round(gx)) < half) | (np.abs(gy - np.round(gy)) < half))
        return np.dstack([np.where(on_line, 210, 40).astype(np.uint8)] * 3)

    def publish_frame(self, frame: np.ndarray) -> None:
        pass

    def send_wheel_speeds(self, fl: float, fr: float, rr: float, rl: float) -> None:
        self.left, self.right = fl, fr

    def get_distance_cm(self) -> Optional[int]:
        radians = math.radians(self.angle)
        fx, fy = math.sin(radians), -math.cos(radians)
        nearest = 200.0
        for ox, oy in self.obstacles:
            vx, vy = ox * SPACING - self.x, oy * SPACING - self.y
            distance = math.hypot(vx, vy)
            if distance > 0 and (vx * fx + vy * fy) / distance >= math.cos(math.radians(30)):
                nearest = min(nearest, distance)
        return int(nearest)


class TurnWatcher:
    """Note où le robot se trouve réellement quand il commence un virage de 90° ou une livraison."""
    stop_requested = False

    def __init__(self, world: RenderedWorld) -> None:
        self.world = world
        self.previous = ""
        self.misplaced: List[Tuple[str, Any, float]] = []

    def publish(self, status: Any) -> None:
        if status.state != self.previous and status.state in ("TURNING_90", "TURNING_MINUS_90", "DELIVERING"):
            nx, ny = status.position
            offset = math.hypot(self.world.x - nx * SPACING, self.world.y - ny * SPACING)
            if offset > SPACING / 4:
                self.misplaced.append((status.state, status.position, round(offset)))
        self.previous = status.state


def _detour(obstacle: float, turn_error: float) -> Tuple[Mission, TurnWatcher]:
    world = RenderedWorld((0, 0), Direction.SOUTH, [(0, obstacle)], turn_error)
    watcher = TurnWatcher(world)
    mission = run_state_machine(world, world, (0, 0), (0, 4), Direction.SOUTH, ImageProcessor(),  # type: ignore[arg-type]
                                watcher, Grid())  # type: ignore[arg-type]
    return mission, watcher


KNOWN_LIMIT = pytest.mark.xfail(
    strict=True,
    reason="Limite connue : après la livraison, trois demi-tours à 3 % cumulent 16° d'erreur. Le robot repart "
           "à 5 cm de la ligne et manque un T en bord de grille. Baisser le seuil rouvrirait les faux croisements.",
)


@pytest.mark.parametrize("obstacle,turn_error", [
    (2.5, 0.92), pytest.param(2.5, 0.97, marks=KNOWN_LIMIT), (2.5, 1.0), (2.5, 1.03), (2.5, 1.08),
    (2.7, 0.92), (2.7, 0.97), (2.7, 1.0), (2.7, 1.03), (2.7, 1.08),
])
def test_jamais_de_virage_en_pleine_ligne_apres_un_contournement(obstacle: float, turn_error: float) -> None:
    """Demi-tour et virage imprécis s'additionnent : le robot ne doit pas y voir un faux croisement."""
    _, watcher = _detour(obstacle, turn_error)
    assert watcher.misplaced == []


def test_contournement_complet_avec_le_vrai_traitement_d_image() -> None:
    mission, watcher = _detour(2.5, 1.0)
    assert mission.destination_reached and watcher.misplaced == []
