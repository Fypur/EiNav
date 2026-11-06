"""Tests de la navigation avec le vrai traitement d'image (images calculées depuis la position du robot, virages imprécis)."""
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
from vision.image_processor import ImageProcessor, far_line_center

SPACING = 60.0  # cm entre deux croisements
GAIN = 150.0  # cm/s à pleine vitesse
LINE_WIDTH = 3.0  # cm
ANGLE = {Direction.NORTH: 0.0, Direction.EAST: 90.0, Direction.SOUTH: 180.0, Direction.WEST: 270.0}
ROWS, COLS = np.mgrid[0:128, 0:160]


# Reconnaissance d'un croisement

def _frame(*polygons: Sequence[Sequence[Tuple[int, int]]]) -> np.ndarray:
    """Polygones blancs sur un sol sombre, coordonnées dans l'image 160 x 128."""
    frame = np.full((128, 160, 3), 40, np.uint8)
    for polygon in polygons:
        cv2.fillPoly(frame, [np.array(polygon, np.int32)], (220, 220, 220))
    return frame


def _intersection(frame: np.ndarray) -> bool:
    return ImageProcessor().process_frame(frame).is_intersection


# Ligne en biais qui passe par un coin de la zone analysée, comme sur les vraies images
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
    """Ligne décalée vers la branche, la bande reste assez large en comptant la ligne."""
    line = [(45, 0), (65, 0), (65, 127), (45, 127)]
    assert _intersection(_frame(line, [(0, 95), (65, 95), (65, 110), (0, 110)]))


def test_t_en_bord_de_grille_avec_la_ligne_decalee_vers_sa_branche() -> None:
    """Si on le rate, le robot a un noeud de décalage pour tout le reste de la mission."""
    line = [(100, 0), (120, 0), (120, 127), (100, 127)]
    assert _intersection(_frame(line, [(100, 95), (160, 95), (160, 110), (100, 110)]))


def test_seuil_reglable_depuis_l_interface() -> None:
    frame = _frame(CENTERED_LINE, [(80, 95), (160, 95), (160, 110), (80, 110)])
    assert _intersection(frame)
    tuning.update({"INTERSECTION_MIN_WIDTH_RATIO": 1.0, "HOUGH_HORIZONTAL_MIN_WIDTH_RATIO": 1.0})
    assert vision_utils.INTERSECTION_MIN_WIDTH_RATIO == 1.0
    assert not _intersection(frame)


# Deuxième méthode : une horizontale et une verticale (Hough)

# Branche qui n'atteint pas le bord de la zone : la méthode forte ne peut pas la voir
SHORT_BRANCH = [(90, 90), (140, 90), (140, 105), (90, 105)]


def _turned(polygon: Sequence[Tuple[int, int]], degrees: float) -> List[Tuple[int, int]]:
    """Tourne un polygone autour du milieu du bas de l'image (robot de travers)."""
    a = math.radians(degrees)
    return [(round(80 + (x - 80) * math.cos(a) - (y - 127) * math.sin(a)),
             round(127 + (x - 80) * math.sin(a) + (y - 127) * math.cos(a))) for x, y in polygon]


def test_croisement_que_la_methode_forte_ne_voit_pas() -> None:
    frame = _frame(CENTERED_LINE, SHORT_BRANCH)
    assert _intersection(frame)
    tuning.update({"HOUGH_HORIZONTAL_MIN_WIDTH_RATIO": 1.0})  # sans Hough, plus de croisement
    assert not _intersection(frame)


@pytest.mark.parametrize("degrees", [-15, -8, 8, 15])
def test_croisement_vu_de_travers(degrees: float) -> None:
    assert _intersection(_frame(_turned(CENTERED_LINE, degrees), _turned(SHORT_BRANCH, degrees)))


@pytest.mark.parametrize("settings", [{"HOUGH_HORIZONTAL_MAX_ANGLE": 5}, {"HOUGH_VERTICAL_MIN_ANGLE": 88}])
def test_seuils_d_angle_reglables(settings: Any) -> None:
    frame = _frame(_turned(CENTERED_LINE, 15), _turned(SHORT_BRANCH, 15))
    assert _intersection(frame)
    tuning.update(settings)
    assert not _intersection(frame)


def test_branche_a_45_degres_n_est_pas_une_horizontale() -> None:
    assert not _intersection(_frame(CENTERED_LINE, [(90, 80), (90, 96), (120, 126), (136, 126)]))


def test_ligne_a_45_degres_n_est_pas_une_verticale() -> None:
    line = [(110, 76), (128, 76), (78, 127), (60, 127)]
    assert not _intersection(_frame(line, [(100, 90), (150, 90), (150, 105), (100, 105)]))


def test_horizontale_seule_n_est_pas_un_croisement() -> None:
    assert not _intersection(_frame([(0, 85), (160, 85), (160, 100), (0, 100)]))


def test_reflet_separe_de_la_ligne_n_est_pas_un_croisement() -> None:
    """Le reflet a de longs bords horizontaux, mais il ne touche pas la ligne."""
    assert not _intersection(_frame(CENTERED_LINE, [(100, 90), (150, 90), (150, 110), (100, 110)]))


@pytest.mark.parametrize("top_x", [100, 120, 140, 160])
def test_ligne_en_biais_n_est_pas_un_croisement(top_x: int) -> None:
    """Ligne droite de plus en plus inclinée : un seul sens, donc jamais une horizontale ET une verticale."""
    assert not _intersection(_frame([(top_x, 76), (top_x + 18, 76), (78, 127), (60, 127)]))


def test_bords_en_escalier_ne_font_pas_une_horizontale() -> None:
    """Le masque a parfois des marches sur les bords de la ligne (images réelles 24, 55, 102)."""
    stairs = [(70, 76), (90, 76), (90, 90), (98, 90), (98, 105), (106, 105), (106, 127), (70, 127)]
    assert not _intersection(_frame(stairs))


def test_grosse_marche_trop_courte_pour_une_horizontale() -> None:
    """Marche de 20 pixels : assez pour Hough, mais pas assez longue pour compter comme une ligne."""
    assert not _intersection(_frame([(70, 76), (90, 76), (90, 100), (110, 100), (110, 127), (70, 127)]))


def test_ligne_dans_un_coin_du_bas_n_est_pas_un_croisement() -> None:
    """Comme l'image réelle 7 : la ligne touche le bas et un côté, avec une large bande, mais rien d'autre."""
    assert not _intersection(_frame([(70, 127), (159, 100), (159, 127)]))


# Options ON/OFF de l'interface. Chaque image n'est vue que par une seule des deux options
CORNER = [(70, 127), (159, 100), (159, 127)]  # bas et côté sans le haut : vu seulement si le haut n'est pas exigé
BOTTOM_T = [(0, 100), (160, 100), (160, 118), (0, 118)]


@pytest.mark.parametrize("require_top,use_hough,corner,short_branch", [
    (True, True, False, True),
    (True, False, False, False),
    (False, True, True, True),
    (False, False, True, False),
])
def test_les_deux_options_sont_independantes(require_top: bool, use_hough: bool, corner: bool, short_branch: bool) -> None:
    tuning.update({"CROSSING_REQUIRE_TOP": require_top, "CROSSING_USE_HOUGH": use_hough})
    assert _intersection(_frame(CORNER)) is corner
    assert _intersection(_frame(CENTERED_LINE, SHORT_BRANCH)) is short_branch


def test_sans_hough_la_methode_forte_marche_toujours() -> None:
    tuning.update({"CROSSING_USE_HOUGH": False})
    assert _intersection(_frame(CENTERED_LINE, [(0, 95), (160, 95), (160, 110), (0, 110)]))


def test_sans_le_haut_la_detection_arrive_sans_ligne_au_dessus() -> None:
    """Bande en bas de la zone, ligne seulement en dessous : le haut n'est pas touché."""
    frame = _frame([(70, 110), (90, 110), (90, 127), (70, 127)], BOTTOM_T)
    tuning.update({"CROSSING_USE_HOUGH": False})
    assert not _intersection(frame)
    tuning.update({"CROSSING_REQUIRE_TOP": False})
    assert _intersection(frame)


def test_ligne_large_ne_fait_pas_une_horizontale() -> None:
    """Hough doit regarder les bords, pas l'intérieur de la ligne."""
    assert not _intersection(_frame([(55, 0), (105, 0), (105, 127), (55, 127)]))


def test_virage_du_parcours_en_8_n_est_pas_un_croisement() -> None:
    """Courbe large comme sur les images réelles 10 à 15."""
    frame = np.full((128, 160, 3), 40, np.uint8)
    cv2.ellipse(frame, (-40, 200), (150, 150), 0, -80, -30, (220, 220, 220), 18)
    assert ImageProcessor().process_frame(frame).cx is not None
    assert not _intersection(frame)


def test_detection_stable_d_une_image_a_l_autre() -> None:
    """HoughLinesP tire ses points au hasard : le résultat ne doit pas changer pour la même image."""
    processor = ImageProcessor()
    with_crossing, without = _frame(CENTERED_LINE, SHORT_BRANCH), _frame(CENTERED_LINE)
    assert all(processor.process_frame(with_crossing).is_intersection for _ in range(100))
    assert not any(processor.process_frame(without).is_intersection for _ in range(100))


# Contournement sur la grille rendue

class RenderedWorld(RobotConnection):
    """Le robot se déplace selon ses roues et la caméra calcule la grille vue depuis sa position."""

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
    """Note où est vraiment le robot quand il commence un virage de 90° ou une livraison."""
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


# Après la livraison, le robot arrive décalé de 7 cm à côté du noeud (0, 3), avec l'obstacle juste derrière.
# Il refait un demi-tour et l'alignement vise la ligne depuis ce décalage : il arrive de biais à 18 cm du noeud.
LIMIT_AFTER_ALIGNMENT = pytest.mark.xfail(
    strict=True, reason="Limite connue : décalage latéral de 7 cm + obstacle juste après un noeud + rotations à 3 %.")


@pytest.mark.parametrize("obstacle,turn_error", [
    (2.5, 0.92), pytest.param(2.5, 0.97, marks=KNOWN_LIMIT), (2.5, 1.0), (2.5, 1.03), (2.5, 1.08),
    (2.7, 0.92), pytest.param(2.7, 0.97, marks=LIMIT_AFTER_ALIGNMENT), (2.7, 1.0), (2.7, 1.03), (2.7, 1.08),
])
def test_jamais_de_virage_en_pleine_ligne_apres_un_contournement(obstacle: float, turn_error: float) -> None:
    """Demi-tour et virage imprécis s'additionnent, le robot ne doit pas y voir un faux croisement."""
    _, watcher = _detour(obstacle, turn_error)
    assert watcher.misplaced == []


@pytest.mark.parametrize("branch", [
    [(0, 95), (160, 95), (160, 110), (0, 110)],  # croix
    [(80, 95), (160, 95), (160, 110), (80, 110)],  # T vers la droite
    [(0, 95), (80, 95), (80, 110), (0, 110)],  # T vers la gauche
])
def test_centre_de_la_ligne_pas_fausse_par_les_branches(branch: List[Tuple[int, int]]) -> None:
    detection = ImageProcessor().process_frame(_frame(CENTERED_LINE, branch))
    assert far_line_center(detection) == pytest.approx(80, abs=2)


def test_centre_de_la_ligne_sous_les_branches() -> None:
    """Rien au-dessus de la bande : on mesure sur la ligne en dessous."""
    line = [(50, 105), (70, 105), (70, 127), (50, 127)]
    detection = ImageProcessor().process_frame(_frame(line, [(0, 85), (160, 85), (160, 105), (0, 105)]))
    assert far_line_center(detection) == pytest.approx(60, abs=2)


def test_centre_de_la_ligne_mesure_au_loin() -> None:
    """Ligne inclinée : on veut sa position en haut de la zone, là où l'angle se voit le mieux."""
    detection = ImageProcessor().process_frame(_frame([(100, 76), (116, 76), (86, 127), (70, 127)]))
    assert far_line_center(detection) == pytest.approx(106, abs=4)


def test_centre_de_la_ligne_tient_compte_de_la_zone() -> None:
    tuning.update({"ROI_LEFT_RATIO": 0.25})
    assert far_line_center(ImageProcessor().process_frame(_frame(CENTERED_LINE))) == pytest.approx(80, abs=2)


def test_centre_de_la_ligne_sans_ligne() -> None:
    assert far_line_center(ImageProcessor().process_frame(_frame())) is None


class AlignedWatcher:
    """Note l'orientation du robot juste après l'alignement et à l'arrivée au noeud."""
    stop_requested = False

    def __init__(self, world: RenderedWorld) -> None:
        self.world = world
        self.after_u_turn: Optional[float] = None
        self.at_node: Optional[float] = None
        self.previous = ""

    def publish(self, status: Any) -> None:
        if self.previous == "ALIGNING_LINE" and status.state != "ALIGNING_LINE":
            self.after_u_turn = self.world.angle
        if status.blocked_edges and status.state == "STOP_ON_INTERSECTION" and self.at_node is None:
            self.at_node = self.world.angle
            self.stop_requested = True
        self.previous = status.state


def _angle_error(angle: Optional[float]) -> float:
    assert angle is not None
    return abs((angle + 180) % 360 - 180)


@pytest.mark.parametrize("obstacle", [1.5, 1.9])
@pytest.mark.parametrize("turn_error", [0.75, 0.85, 0.92, 1.0, 1.08, 1.15, 1.25])
def test_bon_demi_tour_meme_si_la_duree_est_mal_reglee(obstacle: float, turn_error: float) -> None:
    """Une rotation 25 % trop courte ou trop longue laissait le robot 45° de travers, ou perdait la ligne."""
    world = RenderedWorld((0, 0), Direction.SOUTH, [(0, obstacle)], turn_error, max_steps=900)
    watcher = AlignedWatcher(world)
    mission = run_state_machine(world, world, (0, 0), (0, 3), Direction.SOUTH, ImageProcessor(),  # type: ignore[arg-type]
                                watcher, Grid())  # type: ignore[arg-type]
    assert not mission.line_lost
    assert _angle_error(watcher.after_u_turn) < 10.0, "aligné avant de repartir"
    assert _angle_error(watcher.at_node) < 10.0, "toujours droit en arrivant au noeud"


def test_contournement_complet_avec_le_vrai_traitement_d_image() -> None:
    mission, watcher = _detour(2.5, 1.0)
    assert mission.destination_reached and watcher.misplaced == []
